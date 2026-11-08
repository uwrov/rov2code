import pathlib
import subprocess
import os

is_posix = os.name == 'posix'  # mac, linux, etc.
python = 'python3' if is_posix else 'python.exe'

surface_cwd = (pathlib.Path(__file__).parent / 'surface').resolve()
surface_proc = subprocess.Popen([python, 'surface.py'], cwd=surface_cwd, shell=not is_posix)

try:
    surface_proc.wait()
except KeyboardInterrupt:
    pass
