"""Build the energy test from the production runner with host-only device doubles."""
from pathlib import Path

from _project import ROOT

runner = (ROOT / 'src/experiment_runner.cpp').read_text(encoding='utf8')


def method(name, result):
    signature = f'{result} ExperimentRunner::{name}('
    assert runner.count(signature) == 1, signature
    start = runner.index(signature)
    end = runner.index('{', start) + 1
    depth = 1
    while depth:
        depth += (runner[end] == '{') - (runner[end] == '}')
        end += 1
    return runner[start:end]


def production_sections():
    header = (ROOT / 'src/experiment_runner.h').read_text(encoding='utf8')
    first_energy_method = runner.index('bool ExperimentRunner::walkingForeAftAllowsOutput(')
    energy_start = runner.rindex('#if defined(FOOT_SUPPORT_ENERGY) && FOOT_SUPPORT_ENERGY', 0, first_energy_method)
    energy_end = runner.index('void ExperimentRunner::appendWalkingEnergyMetadata(', first_energy_method)
    # The foot members live in one feature block with nested feature guards.
    start = header.index('  foot_phase_prediction::Projection foot_pose_;')
    foot_members = []
    depth = 1
    for line in header[start:].splitlines():
        if line.lstrip().startswith('#if'):
            depth += 1
        elif line.lstrip().startswith('#endif'):
            depth -= 1
            if not depth:
                break
        foot_members.append(line)
    assert depth == 0

    start = header.index('  lateral_energy::CurrentTable walking_energy_current_;')
    end = header.index('  void appendWalkingEnergyMetadata', start)
    selected = runner.index('bool ExperimentRunner::tryTiltPriority(')
    selected_end = runner.index('\nbool ExperimentRunner::tryWeakSideAssist(', selected)
    zero = runner.index('void ExperimentRunner::updateEnergyControlAutonomousAtZeroCross(')
    guard = runner.index('  if (!energy_control_autonomous_mode_', zero)
    guard_end = runner.index('  const uint32_t accepted_cross_ms', guard)
    clock = runner.index('  const uint32_t now_ms =', zero)
    residual = runner.rindex('#if defined(WALKING_TRANSFER_V18)', zero, clock)
    residual_end = runner.index('\n\n  // Capture the latest', residual)
    motion = runner.index('void ExperimentRunner::updateEnergyControlAutonomousMotion(')
    before = runner.index('  const float before_deg =', motion)
    route = runner.index('#if defined(WALKING_TRANSFER_V18)', before)
    route_end = runner.index('\n#if FOOT_EARLY_TRIAL', route)
    crossing_router = runner[route:route_end]
    assert 'cutReturnAidAtZero' in crossing_router
    assert 'if(cut_return_at_zero)interpolated_rate_dps=rate_dps;' in crossing_router

    sections = {
        'FOOT_MEMBERS': '\n'.join(foot_members),
        'ENERGY_MEMBERS': header[start:end],
        'SELECTION': runner[selected:selected_end],
        'CROSSING_GUARD': runner[guard:guard_end],
        'RESIDUAL_CURRENT': runner[residual:residual_end].replace('event.', 'e.').replace('t_test_ms', 't'),
        'CROSSING_ROUTER': crossing_router,
        'ENERGY_METHODS': runner[energy_start:energy_end] + '\n#endif\n',
    }
    for name, result in [('restoreUnsentStallRescue', 'void'), ('dispatchTiltPriority', 'bool'),
                         ('acceptOptionalEarlyMotion', 'void'), ('tryFootSupportCandidate', 'void'),
                         ('beginEnergyControlAutonomousPulse', 'bool'), ('updateEnergyControlAutonomousPulse', 'void'),
                         ('stopActivePulse', 'void'), ('stopMotor', 'void')]:
        sections[name] = method(name, result)
    return sections


def build_runner_fixture():
    fixture = Path(__file__).with_name('runner_fixture.cpp').read_text(encoding='utf8')
    for name, implementation in production_sections().items():
        marker = f'/* PRODUCTION: {name} */'
        assert fixture.count(marker) == 1, marker
        fixture = fixture.replace(marker, implementation)
    return fixture
