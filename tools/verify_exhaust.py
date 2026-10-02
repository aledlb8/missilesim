"""Build/run the exhaust GPU regression and image review with an existing MSVC Ninja build.

Usage: python tools/verify_exhaust.py [--build-dir build/ninja-debug] [--motion]
Requires a completed application build, Python, and an OpenGL 4.5 capable GPU.
Writes review images, measurements, and optional animation frames to build/exhaust-review.
"""
import argparse
import json
from pathlib import Path
import re
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build-dir', default='build/ninja-debug')
    parser.add_argument('--motion', action='store_true')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    build = (root / args.build_dir).resolve()
    output = root / 'build/exhaust-review'
    output.mkdir(parents=True, exist_ok=True)
    commands = json.loads((build / 'compile_commands.json').read_text())
    entry = next(e for e in commands if e['file'].replace('\\', '/').endswith('/main.cpp'))
    compiler = Path(entry['command'].split(' /nologo', 1)[0].strip('"'))
    vcvars = next(p / 'Auxiliary/Build/vcvars64.bat' for p in compiler.parents
                  if (p / 'Auxiliary/Build/vcvars64.bat').exists())
    command = entry['command'].rsplit(' -c ', 1)[0]
    command = re.sub(r'/Fo\S+', '/Foexhaust_review.obj', command)
    command += f' -c "{root / "tools/verify_exhaust.cpp"}"'
    # Core objects now come from missilesim_core.lib in LINK_LIBRARIES.
    # Old per-app .obj files may remain after reconfiguring; linking them would
    # silently mix obsolete class layouts with the current headers.
    folders = ('rendering',) if (build / 'lib/missilesim_core.lib').exists() else ('rendering', 'objects', 'physics', 'sim')
    objects = [str(p.relative_to(build)) for folder in folders
               for p in (build / 'CMakeFiles/MissileSimOpenGL.dir/src' / folder).rglob('*.obj')]
    libraries = next(line.split('=', 1)[1].strip() for line in (build / 'build.ninja').read_text().splitlines()
                     if line.strip().startswith('LINK_LIBRARIES =') and 'glad' in line)
    (build / 'exhaust_review.rsp').write_text(' '.join(f'"{p}"' for p in objects) +
                                            ' exhaust_review.obj ' + libraries)
    batch = output / 'compile.bat'
    batch.write_text(f'@echo off\ncall "{vcvars}" >nul\nif errorlevel 1 exit /b 1\n'
                     f'cd /d "{build}"\n{command}\nif errorlevel 1 exit /b 1\n'
                     'link /nologo /debug /out:bin/ExhaustReview.exe @exhaust_review.rsp\n')
    subprocess.run(['cmd', '/c', str(batch)], cwd=root, check=True)
    run = subprocess.run([str(build / 'bin/ExhaustReview.exe')] + (['--motion'] if args.motion else []),
                         cwd=root, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    print(run.stdout)
    (output / 'verification.txt').write_text(run.stdout)
    run.check_returncode()


if __name__ == '__main__':
    main()
