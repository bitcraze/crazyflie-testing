import json
import os
import time

import cflib
from cflib.bootloader import Bootloader
from cflib.crazyflie import Crazyflie
from cflib.crtp.crtpstack import CRTPPacket
from cflib.crtp.crtpstack import CRTPPort

DIR = os.path.dirname(os.path.realpath(__file__))
SITE_PATH = os.path.join(DIR, '../sites/')


class BCDevice:
    def __init__(self, device):
        cflib.crtp.init_drivers()

        self.type = device['type']
        self.name = device['name']
        self.link_uri = device['radio']
        self.cf = Crazyflie(rw_cache='./cache')
        self.bl = Bootloader(self.link_uri)

    def __str__(self):
        return 'test'

    def firmware_up(self) -> bool:
        ''' Return true if we can contact the (stm32 based) firmware '''
        timeout = 2  # seconds
        link = cflib.crtp.get_link_driver(self.link_uri)

        pk = CRTPPacket()
        pk.set_header(CRTPPort.LINKCTRL, 0)  # Echo channel
        pk.data = b'test'
        link.send_packet(pk)

        ts = time.time()
        while True:
            if time.time() - ts > timeout:
                break

            pk_ack = link.receive_packet(0.1)
            if pk_ack is None:
                continue

            if pk_ack.port != CRTPPort.LINKCTRL or pk_ack.channel != 0:
                continue

            if pk.data == pk_ack.data:
                link.close()
                return True

        link.close()
        return False


def get_devices():
    devices = list()

    site = os.getenv('CRAZY_SITE')
    if site is None:
        raise Exception('No CRAZY_SITE env specified!')

    try:
        path = os.path.join(SITE_PATH, '%s.json' % site)
        f_json = open(path, 'r')
        site_j = json.loads(f_json.read())

        for device in site_j['devices']:
            devices.append(BCDevice(device))
    except Exception:
        raise Exception('Failed to parse json %s!' % path)

    return devices
