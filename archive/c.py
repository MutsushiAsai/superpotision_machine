import asyncio
import re
import sys
from bleak import BleakClient, BleakScanner
from concurrent.futures import ThreadPoolExecutor, wait

MIDI_SERVICE_UUID = "03b80e5a-ede8-4b33-a751-6ce34ec4c700"
MIDI_CHARACTERISTIC_UUID = "7772e5db-3868-4112-a1a9-f2669d106bf3"

target_name = "DEVKitC-BLE-MIDI-([0-9]{2})"
target_address = None
loop = asyncio.get_event_loop()

def _handle_notify(sender, data: bytearray):
    print(', '.join('{:02x}'.format(x) for x in data))

def main():
    devices = asyncio.run(BleakScanner.discover(return_adv=True))
    supported_device = []
    for device, adv_data in devices.values():
        m = re.match(target_name, device.name)
        if not m:
            continue

        if not MIDI_SERVICE_UUID in adv_data.service_uuids:
            print(f'"{device.name}" is not suppported BLE-MIDI', file=sys.stderr)
        
        dev_no = int(m.group(1))
        supported_device.append((dev_no, device))

        print(f"address: {device.address}, name: {device.name}, uuid: {adv_data.service_uuids}")

    async def _handle(dev_no, device):
        try:
            async with BleakClient(device) as client:
                if not client.is_connected:
                    print(f"Failed to connect to {device.name}", file=sys.stderr)
                    return

                await client.start_notify(MIDI_SERVICE_UUID, _handle_notify)

                #for service in client.services:
                #    print(service, service.uuid == BLEMIDI_SERVICE_UUID)
                print('----------------------------')

                import time
                while True:
                    time.sleep(1)


        except Exception as ex:
            print(ex)
               
    with ThreadPoolExecutor(max_workers=4) as pool:
        features = [
            pool.submit(asyncio.run, _handle(dev_no, device))
            for dev_no, device in supported_device
        ]

        wait(features)

main()
