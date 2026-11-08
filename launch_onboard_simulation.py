import pathlib
import subprocess
import os

SURFACE_STATION_IP_ADDRESS = '192.168.0.69'

is_posix = os.name == 'posix'  # mac, linux, etc.
python = 'python3' if is_posix else 'python.exe'

onboard_cwd = (pathlib.Path(__file__).parent / 'onboard').resolve()
onboard_proc = subprocess.Popen([python, 'onboard.py',
                                 '--websocket', f'{SURFACE_STATION_IP_ADDRESS}:8001'], cwd=onboard_cwd, shell=not is_posix)

onboard_proc.wait()
