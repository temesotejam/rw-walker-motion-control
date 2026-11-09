#!/usr/bin/env python3
"""Apply a strictly bounded, logging-only correction to the verified V46s baseline."""
from pathlib import Path
import hashlib
from v46t_current_observation_contract import OLD_PREFIX, NEW_PREFIX, SNAPSHOT, OLD_AGE, NEW_AGE
ROOT=Path(__file__).resolve().parents[1]
OLD_REV='v46s_solver_audit_20260915'
NEW_REV='v46t_current_observation_20260915'

def replace(path,old,new):
    p=ROOT/path;t=p.read_text()
    if t.count(old)!=1:raise ValueError(f'{path}: expected one match: {old[:80]}')
    p.write_text(t.replace(old,new,1))

def main():
    s=(ROOT/'src/experiment_runner.cpp').read_text()
    if hashlib.sha256(s.encode()).hexdigest()!='abed74514dd0c29d494c3172b60efa466553b8015c649e4f5306b472a289d893':
        raise ValueError('V46s runner baseline moved')
    start=s.index('void ExperimentRunner::logSampleNow() {');before,body=s[:start],s[start:]
    for old,new in ((SNAPSHOT,''),(OLD_PREFIX,NEW_PREFIX),(OLD_AGE,NEW_AGE)):
        if body.count(old)!=1:raise ValueError('ambiguous logging patch')
        body=body.replace(old,new,1)
    (ROOT/'src/experiment_runner.cpp').write_text(before+body)
    replace('src/config.h',OLD_REV,NEW_REV)
    replace('tools/replay_v46s_solver_audit.py','from convert_rwlog_to_csv import parse_header, verify_crc','from convert_rwlog_to_csv import parse_header, verify_crc\nfrom v46t_current_observation_contract import normalize_current_observation')
    replace('tools/replay_v46s_solver_audit.py',"text = (ROOT / 'src/experiment_runner.cpp').read_text()","text = normalize_current_observation((ROOT / 'src/experiment_runner.cpp').read_text())")
    for path in ('tools/replay_v46s_solver_audit.py','tools/test_v46s_solver_audit.py'):
        p=ROOT/path;t=p.read_text();needle=".replace('v46s_solver_audit_20260915',"
        if t.count(needle)!=1:raise ValueError(path)
        p.write_text(t.replace(needle,".replace('v46t_current_observation_20260915', 'v46s_solver_audit_20260915')"+needle,1))
    for path in ('tools/test_v46r_fast_solver_control.py','tools/test_v46g_highrate_source_guards.py','tools/test_v46i_task_split_source_guards.py'):
        p=ROOT/path;t=p.read_text().replace(OLD_REV,NEW_REV)
        t=t.replace('0.46.18','0.46.19').replace('AtomS3R V46s Fast Solver Motor Validation','AtomS3R V46t Fast Solver Motor Validation')
        if path.endswith('test_v46r_fast_solver_control.py'):t=t.replace('V46s','V46t')
        p.write_text(t)
    p=ROOT/'site/manifest.json';p.write_text(p.read_text().replace('0.46.18','0.46.19').replace('V46s','V46t'))
    p=ROOT/'site/index.html';t=p.read_text().replace('V46s / 0.46.18','V46t / 0.46.19').replace('へV46sを書き込む','へV46tを書き込む').replace('V46s Fast Solver','V46t Fast Solver').replace('/ V46s /','/ V46t /').replace(' V46s ファームウェア',' V46t ファームウェア').replace('AtomS3R V46s','AtomS3R V46t')
    t=t.replace('      <h2>V46t / 0.46.19：高速ソルバの計測とオフライン検証</h2>','''      <h2>V46t / 0.46.19：電流記録の時刻整合を修正</h2>
      <p>電流値・連番・取得時刻を1回のスナップショットから読み、その後に行の参照時刻を採ります。
      別スナップショットの再読出しによる年齢の不整合を除去しました。未取得は従来どおりUINT32_MAXです。
      V46p/V46q/V46sで再現した記録上の問題への修正で、制御判断・MEKF・10 ms停止条件は変更していません。</p>
      <p>角度の積分座標のずれとIMUの取得間隔の揺らぎは別課題です。この版で修正済みとは扱いません。
      V46tの実機確認は未実施です。<a href="https://github.com/temesotejam/atoms3r-mekf-dynamic-validation/blob/main/docs/V46T_CURRENT_OBSERVATION_FIX.md">4 Run比較と修正範囲</a></p>''')
    p.write_text(t)
    replace('src/psram_logger.cpp','  json += "\\\"v46s_solver_audit\\\":";', '  json += "\\\"v46t_current_observation_policy\\\":\\\"single_snapshot_before_row_reference_clock;age_from_same_snapshot;zero_sample_time_is_missing;no_clamp\\\",";\n  json += "\\\"v46s_solver_audit\\\":";')
    print('Applied V46t logging-only fix; no controller, driver, MEKF or acquisition changes')
if __name__=='__main__':main()
