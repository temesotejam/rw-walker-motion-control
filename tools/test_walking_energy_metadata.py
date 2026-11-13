"""Compile actual v14 metadata serializers and round-trip their compact JSON.

Host Arduino/PSRAM allocation doubles verify formatting and schema, not ESP
allocation, scheduler timing or physical walking. Energy decisions, record
members, CompactJsonTable and appendWalkingEnergyMetadata come from real code.
New nominal/neutral diagnostics are copied from source and remain diagnostic;
the assertions below do not claim an identified physical body inertia/model.
"""
from pathlib import Path
import csv
import json
import subprocess
import tempfile

import convert_rwlog_to_csv as converter

ROOT = Path(__file__).resolve().parents[1]


def unique_object(items):
    keys = [key for key, _ in items]
    assert len(keys) == len(set(keys)), f"Duplicate JSON fields: {keys}"
    return dict(items)


def parse(text):
    return json.loads(text, object_pairs_hook=unique_object,
                      parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))


logger = (ROOT / 'src/psram_logger.cpp').read_text(encoding='utf8')
start = logger.index('    if(walking_energy_mode){')
end = logger.index('    }else{', start)
work_metadata = logger[start + len('    if(walking_energy_mode){'):end]
source = (ROOT / 'src/walking_energy_runner.inc').read_text(encoding='utf8')
start = source.index('void ExperimentRunner::appendWalkingEnergyMetadata(')
end = source.index('\n}\n', start) + 3
serializer = source[start:end]
header = (ROOT / 'src/experiment_runner.h').read_text(encoding='utf8')
start = header.index('  struct WalkingEnergyAudit {')
end = header.index('\n  };', start) + len('\n  };')
audit = header[start:end]

fixture = r'''
#include <Arduino.h>
#include "psram_string.h"
#include "compact_json_table.h"
#include "walking_energy_policy.h"
#include <iostream>
#include <iomanip>
#include <cstdlib>
#define CHECK(x) do {if(!(x)){std::cerr<<"check failed: "<<#x<<" line "<<__LINE__<<"\n";std::exit(2);}}while(false)
class ExperimentRunner {public:
AUDIT_MEMBERS
  WalkingEnergyAudit walking_energy_audit_[192];
  uint32_t walking_energy_count_=0,walking_energy_total_=0;
  void appendWalkingEnergyMetadata(PsramString& json) const;
};
SERIALIZER
int main(int argc,char** argv){
  ExperimentRunner runner;
  const bool empty=argc>1 && std::string(argv[1])=="empty";
  lateral_energy::Geometry geometry;
  lateral_energy::CurrentTable table;CHECK(table.prepare(.0316f));
  for(unsigned i=0;i<4 && !empty;++i){
    walking_energy_policy::Input in;
    in.side=i==2?-1:1;in.rate_dps=i==1?80.f:i==2?5.5f:55.f;
    in.body_angle_deg=i==2?3.f:0.f;in.target_deg=8.f;
    in.previous_peak_deg=8.f;in.same_side_peak_deg=i==2?NAN:6.f;
    in.initial_current_ma=i==2?-150.f:-5.f;in.goal_current_ma=329.547119f;
    in.cap_ms=i==2?15:80;
    auto selected=walking_energy_policy::decide(geometry,table,in);
    CHECK(selected.valid);
    auto& a=runner.walking_energy_audit_[i];
    a.time_ms=1200+i*300;a.peak_ms=a.time_ms-250;a.side=in.side;
    a.requested_ms=selected.width_ms;
    a.guarded_ms=i==0?selected.width_ms/2:selected.width_ms;
    a.decision=walking_energy_policy::evaluateWidth(geometry,table,in,a.guarded_ms);
    a.optional=i==2;a.rescue=i==2;
    a.executed=a.decision.valid && !a.decision.coast;
    if(i==3){a.decision=walking_energy_policy::Decision{};a.executed=false;}
    ++runner.walking_energy_count_;++runner.walking_energy_total_;
  }
  if(!empty){CHECK(!runner.walking_energy_audit_[0].decision.coast);
    CHECK(runner.walking_energy_audit_[1].decision.coast);
    CHECK(runner.walking_energy_audit_[2].decision.passive_intercept_rolloff);}
  PsramString json;CHECK(json.reserve(1024*1024));json+="{";
WORK_METADATA
  json+="\"sentinel\":0";
  runner.appendWalkingEnergyMetadata(json);json+="}";CHECK(json.ok());
  std::cout<<json.c_str()<<"\n[";
  // Independent high-precision values expose truncation of sub-mJ terms.
  for(unsigned i=0;i<runner.walking_energy_count_;++i){
    if(i)std::cout<<",";
    const auto& d=runner.walking_energy_audit_[i].decision;
    std::cout<<"{";bool first=true;
    auto number=[&](const char* key,float value){if(!first)std::cout<<",";first=false;
      std::cout<<"\""<<key<<"\":";
      if(std::isfinite(value))std::cout<<std::setprecision(17)<<value;else std::cout<<"null";};
    number("available_j",d.available_j);number("target_j",d.target_j);
    number("predicted_work_j",d.predicted_work_j);number("mean_peak_energy_j",d.mean_peak_energy_j);
    number("redistribution_j",d.redistribution_j);number("request_j",d.request_j);
    std::cout<<"}";
  }
  std::cout<<"]\n";
  // The real compact-table encoder rejects drift instead of silently
  // associating values with a previous row's different columns.
  PsramString bad;CHECK(bad.reserve(1024));CompactJsonTable<PsramString> compact(bad);
  CHECK(compact.append("{\"a\":1}"));CHECK(!compact.append("{\"b\":2}"));
  return 0;
}
'''.replace('AUDIT_MEMBERS', audit).replace('SERIALIZER', serializer).replace('WORK_METADATA', work_metadata)

with tempfile.TemporaryDirectory(prefix='walking_energy_metadata_') as temp:
    temp = Path(temp)
    (temp / 'test.cpp').write_text(fixture, encoding='utf8')
    exe = temp / 'test.exe'
    subprocess.run(['g++', '-std=c++17', '-O2', '-Wall', '-Wextra',
                    '-I' + str(ROOT / 'tools/host_v46o'), '-I' + str(ROOT / 'src'),
                    str(temp / 'test.cpp'), '-o', str(exe)], check=True)
    defines = ['FOOT_EARLY_TRIAL', 'ANGLE_EARLY_TRIAL', 'WALK_SMOOTH_TRIAL',
               'SYMMETRIC_RESCUE_TRIAL', 'EARLY_RESCUE_TRIAL', 'SWAY_BALANCE_TRIAL',
               'FOOT_SUPPORT_ASSIST', 'FOOT_SUPPORT_LOCAL', 'FOOT_SUPPORT_TRANSFER']
    for energy, expected in [(False, 'foot-support-transfer-v13'), (True, 'foot-support-energy-v14')]:
        flags = ['-D' + name + '=1' for name in defines + (['FOOT_SUPPORT_ENERGY'] if energy else [])]
        macros = subprocess.check_output(['g++', '-E', '-dM', '-x', 'c++',
                                          str(ROOT / 'src/runtime_diagnostics.h')] + flags, text=True)
        assert '#define RUNTIME_DIAGNOSTIC_BUILD_ID "' + expected + '"' in macros
    for mode, expected_count in [('', 4), ('empty', 0)]:
        lines = subprocess.check_output([str(exe)] + ([mode] if mode else []), text=True).splitlines()
        assert len(lines) == 2
        raw, reference = map(parse, lines)
        assert raw['wheel_work_model_revision'] == 'walking_peak_rate_energy_v14'
        assert raw['work_control_branch_codes']['19'] == 'walking_peak_rate_energy_v14'
        for key in ['tilt_priority', 'foot_span_control', 'foot_span_high_current',
                    'positive_sway_probe', 'weak_side_assist']:
            assert raw[key]['active'] is False and raw[key]['width_sizing_active'] is False
        assert raw['walking_passive_fit']['plus_rate_support_dps'] == [22.2512, 37.5379]
        assert raw['walking_passive_fit']['powered_response_transfer_validated'] is False
        block = raw['walking_energy']
        assert block['revision'] == 'walking_peak_rate_energy_v14'
        assert block['physical_body_inertia_identified'] is False and block['MPC_active'] is False
        assert block['coast_residual_modelled'] is False
        records = block['records']
        assert set(records) == {'fields', 'rows'}
        assert len(records['fields']) == len(set(records['fields']))
        assert len(records['rows']) == expected_count
        assert all(len(row) == len(records['fields']) for row in records['rows'])
        assert block['stored'] == block['total'] == expected_count and block['truncated'] is False
        metadata = converter.expand_tables(raw)
        rows = metadata['walking_energy']['records']
        for row, original in zip(rows, reference):
            for key, value in original.items():
                if value is None:
                    assert row[key] is None
                else:
                    assert abs(row[key] - value) <= 5.01e-10, (key, row[key], value)
        if rows:
            assert rows[0]['predicted_work_j'] > 0 and rows[0]['guarded_ms'] < rows[0]['requested_ms']
            assert rows[1]['coast'] and rows[1]['q_mas'] == rows[1]['predicted_work_j'] == 0
            assert rows[2]['rescue'] and rows[2]['passive_intercept_rolloff'] and rows[2]['extrapolated']
            assert not rows[3]['valid'] and rows[3]['available_j'] is None
        out = temp / ('csv_' + (mode or 'records'))
        out.mkdir()
        assert converter.write_walking_energy_diagnostics(metadata, out) == expected_count
        csv_path = out / 'walking_energy.csv'
        if not rows:
            assert not csv_path.exists()
            continue
        with csv_path.open(newline='', encoding='utf8') as stream:
            csv_rows = list(csv.DictReader(stream))
        assert len(csv_rows) == len(rows)
        for csv_row, json_row in zip(csv_rows, rows):
            assert set(csv_row) == set(json_row)
            for key, value in json_row.items():
                if value is None:
                    assert csv_row[key] == ''
                elif isinstance(value, bool):
                    assert csv_row[key] == str(value)
                elif isinstance(value, (int, float)):
                    assert float(csv_row[key]) == value, (key, csv_row[key], value)
                else:
                    assert csv_row[key] == value

print('walking metadata: actual C++ JSON/compact rows/legacy flags/J precision/CSV roundtrip PASS')
