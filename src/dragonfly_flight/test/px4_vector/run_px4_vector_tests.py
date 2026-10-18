import os
"""Link focused controller checks with the existing PX4 SITL build.

Requires the system GoogleTest development library. No simulator is launched.
"""
from pathlib import Path
import argparse
import shlex
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument('--px4', type=Path, default=Path(os.environ.get('PX4_ROOT', str(Path.home()/'PX4-Autopilot'))))
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--source', type=Path)
args = parser.parse_args()
build = args.px4 / 'build/px4_sitl_default'
args.output.mkdir(parents=True, exist_ok=True)
obj = args.output.resolve() / 'test_px4_vector.o'
exe = args.output.resolve() / 'test_px4_vector'
source = (args.source or Path(__file__).with_name('test_px4_vector.cpp')).resolve()

def commands(target):
    result = subprocess.check_output(['ninja', '-t', 'commands', target], cwd=build, text=True)
    return result.strip().splitlines()

compile_line = commands('src/modules/mc_pos_control/PositionControl/CMakeFiles/PositionControl.dir/PositionControl.cpp.o')[-1]
tokens = shlex.split(compile_line)
compile_args = []
i = 0
while i < len(tokens):
    if tokens[i] in ('-o', '-c', '-MF', '-MT'):
        i += 2
        continue
    if tokens[i] in ('-MD',):
        i += 1
        continue
    compile_args.append(tokens[i])
    i += 1
compile_args += ['-I' + str(args.px4 / 'src/lib/control_allocation/control_allocation'), '-c', str(source), '-o', str(obj)]
subprocess.run(compile_args, cwd=build, check=True, timeout=120)

tokens = shlex.split(commands('bin/px4')[-1])
start = next(i for i, token in enumerate(tokens) if Path(token).name in ('c++', 'g++', 'clang++'))
end = tokens.index('&&', start) if '&&' in tokens[start:] else len(tokens)
link_args = tokens[start:end]
for i, token in enumerate(link_args):
    if token.endswith('/src/px4/common/main.cpp.o'):
        link_args[i] = str(obj)
    if token == '-o':
        link_args[i + 1] = str(exe)
link_args += ['-lgtest', '-lgtest_main', '-pthread']
subprocess.run(link_args, cwd=build, check=True, timeout=120)
subprocess.run([str(exe), '--gtest_output=xml:' + str(args.output.resolve() / 'tests.xml')], cwd=args.output, check=True, timeout=30)
