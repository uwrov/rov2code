import argparse
import asyncio
import json
import signal
import sys
import time

import websockets

from rov_base import ROVBase

COMMAND_TIMEOUT = 1.0  # seconds without a command before everything stops

# SIGHUP is what a closed SSH session sends
SHUTDOWN_SIGNALS = [getattr(signal, name) for name in ('SIGTERM', 'SIGHUP') if hasattr(signal, name)]

def exit_on_signal(signum, frame):
    for shutdown_signal in SHUTDOWN_SIGNALS:
        signal.signal(shutdown_signal, signal.SIG_IGN)  # don't interrupt stop_all
    sys.exit(128 + signum)


def main():
    print('onboard client starting')
    for shutdown_signal in SHUTDOWN_SIGNALS:
        signal.signal(shutdown_signal, exit_on_signal)

    rov, websocket_uri = setup_using_command_line_args()
    try:
        asyncio.run(client_handler(websocket_uri, rov))
    finally:
        rov.stop_all()


# allows file to be run with arguments
# includes default websocket if none specified 
def setup_using_command_line_args():
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument('-p', '--physical', action='store_true',
                            help='run using physical instead of simulated hardware')
    arg_parser.add_argument('-w', '--websocket', default='localhost:8001',
                            help='websocket\'s ip and port, e.g. localhost:8001')
    args = arg_parser.parse_args()

    if args.physical:
        from physical.physical import ROV
    else:
        from simulated.simulated import ROV

    rov = ROV()
    websocket_uri = f'ws://{args.websocket}'
    
    return rov, websocket_uri


# stops everything if commands stop arriving, even if the connection still looks open
class CommandWatchdog:
    def __init__(self, rov: ROVBase):
        self.rov = rov
        self.last_command = time.monotonic()
        self.stopped = False

    def feed(self):
        self.last_command = time.monotonic()
        self.stopped = False

    def check_timeout(self):
        if not self.stopped and time.monotonic() - self.last_command > COMMAND_TIMEOUT:
            self.stopped = self.rov.stop_all()  # stays False to retry if a pin failed
            print('onboard: no commands received - stopping thrusters')

    async def run(self):
        while True:
            await asyncio.sleep(COMMAND_TIMEOUT / 4)
            try:
                self.check_timeout()
            except Exception:
                pass


# connect to surface station
async def client_handler(websocket_uri: str, rov: ROVBase):
    watchdog = CommandWatchdog(rov)
    watchdog_task = asyncio.create_task(watchdog.run())
    async for websocket in websockets.connect(websocket_uri):
        try:
            await asyncio.gather(
                consumer_handler(websocket, rov, watchdog),
                producer_handler(websocket, rov)
            )
        except websockets.ConnectionClosed:
            rov.stop_all()
            print('onboard: websockets.ConnectionClosed - stopping thrusters and retrying...')
            await asyncio.sleep(1)


# incoming command handling (instructions for ROV)
async def consumer_handler(websocket: 'ClientConnection', rov: ROVBase, watchdog: CommandWatchdog):
    async for message in websocket:
        try:
            data = json.loads(message)
            if data['type'] == 'set_pin_pwms':
                for pin in data['pins']:
                    rov.set_pin_pwm(pin['number'], pin['value'])
                await rov.flush_pin_pwms()
                watchdog.feed()
            else:
                print(f"Invalid type {data['type']} for message: {message}")
        except Exception as e:
            print(f'onboard: error handling message: {e!r}')


# outgoing data handling (sensor readings)
async def producer_handler(websocket: 'ClientConnection', rov: ROVBase):
    while True:
        await asyncio.gather(
            send_onboard_digest(websocket, rov),
            asyncio.sleep(0.1)  # limit to 10 summaries per second
        )

# TODO: create hardware-agnostic sensor reading system
# currently requires hardcoding sensor pins, protocols, etc.
async def send_onboard_digest(websocket: 'ClientConnection', rov: ROVBase):
    try:
        readings = await rov.poll_sensors()
    except Exception as e:
        print(f'onboard: sensor poll failed: {e!r}')
        return

    await websocket.send(json.dumps({
        'type': 'sensor_summary',
        **readings,
    }))


if __name__ == "__main__":
    main()
