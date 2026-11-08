import pathlib
import subprocess

vacuum = input(f"Has ROV passed a vacuum test since last opening? y/n: ")
if (vacuum.lower() != 'y'):
	raise Exception("pull vacuum to -14psi and hold for 10 minutes.")
guards = input(f"Are all 12 thruster guards present and undamaged? y/n: ")
if (guards.lower() != "y"):
	raise Exception("install replacement guards. Can be found on cart mini-shelf")
rigid = input(f"Are all 6 thrusters mounted rigidly to the frame? y/n: ")
if (rigid.lower() != "y"):
	raise Exception("Tighten screws, or replace motor mount. Replacements can be found on cart.")


# our ROV uses a raspberry pi (linux), so we don't need to have windows-specific workarounds for physical onboard launch

SURFACE_STATION_IP_ADDRESS = '172.25.250.2'

onboard_cwd = (pathlib.Path(__file__).parent / 'onboard').resolve()
onboard_proc = subprocess.Popen(['python3', 'onboard.py', '--physical',
                                '--websocket', f'{SURFACE_STATION_IP_ADDRESS}:8001'], cwd=onboard_cwd)

onboard_proc.wait()
