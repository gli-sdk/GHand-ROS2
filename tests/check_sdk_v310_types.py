"""Compile a contract comparison against a supplied GHand-Cpp-SDK V3.1.0 checkout."""

import argparse
from pathlib import Path
import re
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]


def check(sdk):
    include = ROOT / "ghand_hardware/ghand_hardware_interface/include"
    local = (include / "ghand_hardware_interface/communication/sdk_types.hpp").read_text()
    lines = [
        '#include "ghand/types/data.h"',
        '#include "ghand/version.h"',
        '#include "ghand_hardware_interface/communication/sdk_types.hpp"',
        '#include <algorithm>', '#include <cassert>', '#include <cstddef>',
        '#include <type_traits>',
        'static_assert(GHAND_VERSION_MAJOR == 3 && GHAND_VERSION_MINOR == 1 && GHAND_VERSION_PATCH == 0);',
    ]
    for kind, body in re.findall(r'enum class (\w+)\s*:\s*\w+\s*\{(.*?)\};', local, re.S):
        body = re.sub(r'//[^\n]*', '', body)
        for item in body.split(','):
            name = item.strip().split('=')[0].strip()
            if name:
                lines.append(f'static_assert(static_cast<int>(ghand::{kind}::{name}) == '
                             f'static_cast<int>(ghand_comm::{kind}::{name}));')
    fields = {
        'JointData': ['id', 'state', 'error', 'control_mode', 'loop_type', 'angle', 'velocity', 'current'],
        'JointCommand': ['id', 'control_mode', 'loop_type', 'control_state', 'angle', 'velocity', 'current', 'kp', 'kd'],
        'HandData': ['joints_state', 'error_warn', 'error_fault', 'error_critical', 'temperature'],
        'TactileData': ['state', 'resultant_force', 'distributed_force'],
        'DeviceData': ['hand', 'joints', 'tactile', 'timestamp'],
    }
    for kind, names in fields.items():
        lines.append(f'static_assert(sizeof(ghand::{kind}) == sizeof(ghand_comm::{kind}));')
        lines.append(f'static_assert(alignof(ghand::{kind}) == alignof(ghand_comm::{kind}));')
        for name in names:
            lines.append(f'static_assert(offsetof(ghand::{kind}, {name}) == '
                         f'offsetof(ghand_comm::{kind}, {name}));')
    for kind in ('JointCommand', 'JointData'):
        for name in ('angle', 'velocity', 'current'):
            lines.append(f'static_assert(std::is_same_v<decltype(ghand::{kind}::{name}), '
                         f'decltype(ghand_comm::{kind}::{name})>);')
    lines.append('int main() {')
    for kind in ('JointData', 'JointCommand'):
        lines.append(f'{{ const ghand::{kind} sdk; const ghand_comm::{kind} ros;')
        for name in fields[kind]:
            lines.append(f'assert(static_cast<double>(sdk.{name}) == static_cast<double>(ros.{name}));')
        lines.append('}')
    for kind in ('HandWarning', 'HandFault', 'HandCriticalError'):
        names = re.findall(r'static const ' + kind + r' (\w+);', local)
        for name in names:
            lines.append(f'assert(std::equal(ghand::{kind}::{name}.begin(), '
                         f'ghand::{kind}::{name}.end(), ghand_comm::{kind}::{name}.begin()));')
    lines.append('}')
    with tempfile.TemporaryDirectory() as directory:
        source = Path(directory) / 'sdk_contract.cpp'
        binary = Path(directory) / 'sdk_contract'
        source.write_text('\n'.join(lines))
        subprocess.run([
            'c++', '-std=c++17', '-Wno-invalid-offsetof', '-I', str(include),
            '-I', str(sdk / 'include'), str(source), str(sdk / 'src/types.cc'),
            '-o', str(binary),
        ], check=True)
        subprocess.run([str(binary)], check=True)
    print('PASS: SDK V3.1.0 data layouts, enum values, joint defaults, and all hand error flags')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('sdk_root', type=Path)
    check(parser.parse_args().sdk_root.resolve())
