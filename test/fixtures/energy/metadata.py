"""Exercise production energy metadata serialization on a host-side logger fixture."""
import json
from pathlib import Path
import re

from _project import ROOT

logger = (ROOT / 'src/psram_logger.cpp').read_text(encoding='utf8')


def unique_object(items):
    keys = [key for key, _ in items]
    assert len(keys) == len(set(keys)), f"Duplicate JSON fields: {keys}"
    return dict(items)


def parse(text):
    return json.loads(text, object_pairs_hook=unique_object,
                      parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))


def revision_block(source, start):
    """Read one balanced conditional, including nested version selections."""
    depth = 0
    offset = start
    for line in source[start:].splitlines(keepends=True):
        directive = re.match(r'\s*#\s*(if|ifdef|ifndef|endif)\b', line)
        if directive:
            depth += -1 if directive[1] == 'endif' else 1
            assert depth >= 0, 'Unbalanced revision conditional'
        offset += len(line)
        if directive and depth == 0:
            return source[start:offset]
    raise AssertionError('Unterminated revision conditional')


def production_sections():
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

    def statement(field):
        marker = '  json += "\\\"' + field + '\\\"'
        start = logger.index(marker)
        return logger[start:logger.index(';\n', start) + 2]

    fields = [
        'energy_model', 'energy_control_autonomous_policy', 'lateral_energy_revision',
        'lateral_energy_formula', 'lateral_energy_coefficient_status', 'energy_control_autonomous_revision',
        'energy_request_audit_revision', 'energy_request_audit_semantics', 'energy_control_autonomous_free_model',
        'energy_control_autonomous_q_policy',
    ]
    start = logger.index('    detail += ",\\\"free_model_revision\\\"')
    end = logger.index('    detail += ",\\\"work_control_branch\\\"', start)
    revision_code = logger[start:end]
    maximum_marker = '    appendAutonomousNullable("maximum_input_work_j",'
    maximum_start = logger.index(maximum_marker)
    maximum_code = logger[maximum_start:logger.index(';\n', maximum_start) + 2]
    revision_start = logger.index('#if defined(WALKING_TRANSFER_V18) && WALKING_TRANSFER_V18\n  const char* walking_energy_revision=')

    return {
        'AUDIT_MEMBERS': audit,
        'SERIALIZER': serializer,
        'WORK_METADATA': work_metadata,
        'POLICY_FIELDS': ''.join(statement(key) for key in fields),
        'EVENT_REVISION': revision_code,
        'MAXIMUM_WORK': maximum_code,
        'MODEL_REVISION': revision_block(logger, revision_start),
    }


def build_metadata_fixture():
    fixture = Path(__file__).with_name('metadata_fixture.cpp').read_text(encoding='utf8')
    for name, implementation in production_sections().items():
        marker = f'/* PRODUCTION: {name} */'
        assert fixture.count(marker) == 1, marker
        fixture = fixture.replace(marker, implementation)
    return fixture
