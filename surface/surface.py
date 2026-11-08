import asyncio
import json

from core.core import Core, TIME_PER_CYCLE
from interface.interface import Interface

import websockets
import functools

def main():
    print('surface server starting')
    core = Core()

    interface = Interface(core)
    core.set_interface(interface)

    print('serving')
    asyncio.run(serve(core, interface))

# start the surface station and interface servers
# rov_server communicates with the actual ROV
# interface_server communicates with Godot interface running on surface station
# gather makes an infinite loop to keep communicating with ROV and interface
async def serve(core: Core, interface: Interface):
    rov_server = await websockets.serve(
        functools.partial(rov_server_handler, core=core),
        "", 8001
    )
    interface_server = await websockets.serve(interface.server_handler, "", 8002)
    video_task = asyncio.create_task(interface.video_loop())
    try:
        await asyncio.gather(
            rov_server.wait_closed(),
            interface_server.wait_closed(),
            video_task
        )
    except Exception as e:
        print(repr(e))
        await asyncio.gather(
            rov_server.wait_closed(),
            interface_server.wait_closed(),
        )


# handles incoming ("consumer") and outgoing ("producer") communication with ROV
# see: https://websockets.readthedocs.io/en/stable/howto/patterns.html
async def rov_server_handler(websocket: 'ClientConnection', core: Core):
    await asyncio.gather(
        consume_incoming_data_from_rov(websocket, core),
        produce_outgoing_commands_to_rov(websocket, core)
    )


async def consume_incoming_data_from_rov(websocket: 'ClientConnection', core: Core):
    async for message in websocket:
        # A bad packet or a failed Godot update shouldn't drop the ROV link
        try:
            data = json.loads(message)
            if data['type'] == 'sensor_summary':
                await core.update_sensors(data)
            else:
                print(f"Invalid type {data['type']} for message: {message}")
        except Exception as e:
            print(f'surface: error handling ROV message: {e!r}')


async def produce_outgoing_commands_to_rov(websocket: 'ClientConnection', core: Core):
    while True:
        await asyncio.gather(
            update_controls_and_send_to_rov(websocket, core),
            asyncio.sleep(TIME_PER_CYCLE)  # limit to 10 summaries per second
        )


async def update_controls_and_send_to_rov(websocket: 'ClientConnection', core: Core):
    pin_pwms = await core.update_controls()
    await websocket.send(json.dumps({
        'type': 'set_pin_pwms',
        'pins': pin_pwms
    }))


if __name__ == "__main__":
    main()
