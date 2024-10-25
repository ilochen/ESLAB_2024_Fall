# ble_scan_connect.py

from bluepy.btle import Peripheral, UUID
from bluepy.btle import Scanner, DefaultDelegate
from bluepy import btle

class ScanDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)

    def handleDiscovery(self, dev, isNewDev, isNewData):
        if isNewDev:
            print("Discovered device", dev.addr)
        elif isNewData:
            print("Received new data from", dev.addr)
            
class MyDelegate(btle.DefaultDelegate):
    def __init__(self):
        btle.DefaultDelegate.__init__(self)

    def handleNotification(self, cHandle, data):
        print("A notification was received: %s" %data)


scanner = Scanner().withDelegate(ScanDelegate())
devices = scanner.scan(10.0)

n = 0
addr = []
done = False
for dev in devices:
    print(f"{n}: Device {dev.addr} ({dev.addrType}), RSSI={dev.rssi} dB")
    print(dev.getScanData())
    addr.append(dev.addr)
    n += 1
    for (adtype, desc, value) in dev.getScanData():
        print(f" {desc} = {value}")

number = input('Enter your device number: ')
print('Device', number)
num = int(number)
print(addr[num])

print("Connecting...")
dev = Peripheral(addr[num], 'random')
dev.setDelegate( MyDelegate() )


print("Services...")
for svc in  dev.getServices() :
    print(str(svc))
    for ch in svc.getCharacteristics():
        print(str(ch))
        print(str(ch.uuid))
        #if(ch.supportsRead()):
        #    ch.read()
        if(str(ch.uuid) == "00e00000-0001-11e1-ac36-0002a5d5c51b"):
            print("Found ACC")
            print("ACC handle = ", ch.getHandle())
            print(ch.read())
            if(len(ch.getDescriptors()) == 0):
                    continue
            for desc in ch.getDescriptors():
                    print(str(desc))
                    if desc.uuid == btle.AssignedNumbers.client_characteristic_configuration:
                        print(f"Found CCCD for characteristic ACC: {desc.uuid}")
                        print(desc.read())
                        desc.write(b"\x01\x00", withResponse=True)
                        while True:
                            if dev.waitForNotifications(1.0):
                            # handleNotification() was called
                                break
                            print("Waiting...")
                           
                        print(f"Indications enabled for characteristic ACC")
                        print(desc.read())
                        

                        #if ch.properties & btle.Characteristic.WRITE:
        # if(str(ch.uuid) == "00140000-0001-11e1-ac36-0002a5d5c51b"):
        #     print("found ENV")
        #     print("ENV handle = ", ch.getHandle())
        #     ch.write(b"\x22", withResponse=True)
        #     print(ch.read())
        #     if(len(ch.getDescriptors()) == 0):
        #             continue
        #     for desc in ch.getDescriptors():
        #             print(str(desc))
        #             print(desc.read())
        # if(str(ch.uuid) == " 00e00000-0001-11e1-ac36-0002a5d5c51c"): 
        #     print("found FREQ")
        #     print("FREQ handle = ", ch.getHandle())
        #     ch.write(b"\x22", withResponse=True)
        #     print(ch.read())
        #     if(len(ch.getDescriptors()) == 0):
        #             continue
        #     for desc in ch.getDescriptors():
        #             print(str(desc))
        #             print(desc.read())          
                    
dev.disconnect()
