import argparse
import asyncio
import json
import time

import websockets

COMMAND_TIMEOUT = 1.0  # seconds without a command before everything stops


def main():
    print('onboard client starting')
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
    def __init__(self, rov: 'ROV'):
        self.rov = rov
        self.last_command = time.monotonic()
        self.stopped = False

    def feed(self):
        self.last_command = time.monotonic()
        self.stopped = False

    async def run(self):
        while True:
            await asyncio.sleep(COMMAND_TIMEOUT / 4)
            if not self.stopped and time.monotonic() - self.last_command > COMMAND_TIMEOUT:
                print('onboard: no commands received - stopping thrusters')
                self.rov.stop_all()
                self.stopped = True


# connect to surface station
async def client_handler(websocket_uri: str, rov: 'ROV'):
    watchdog = CommandWatchdog(rov)
    watchdog_task = asyncio.create_task(watchdog.run())
    async for websocket in websockets.connect(websocket_uri):
        try:
            await asyncio.gather(
                consumer_handler(websocket, rov, watchdog),
                producer_handler(websocket, rov)
            )
        except websockets.ConnectionClosed:
            print('onboard: websockets.ConnectionClosed - stopping thrusters and retrying...')
            rov.stop_all()
            await asyncio.sleep(1)
            continue


# incoming command handling (instructions for ROV)
async def consumer_handler(websocket: 'ClientConnection', rov: 'ROV', watchdog: CommandWatchdog):
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
async def producer_handler(websocket: 'ClientConnection', rov: 'ROV'):
    while True:
        await asyncio.gather(
            send_onboard_digest(websocket, rov),
            asyncio.sleep(0.1)  # limit to 10 summaries per second
        )

# TODO: create hardware-agnostic sensor reading system
# currently requires hardcoding sensor pins, protocols, etc.
async def send_onboard_digest(websocket: 'ClientConnection', rov: 'ROV'):
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
