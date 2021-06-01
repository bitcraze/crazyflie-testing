import os
import time
import toml
import glob

import cflib
from cflib.bootloader import Bootloader
from cflib.crazyflie import Crazyflie

DIR = os.path.dirname(os.path.realpath(__file__))
SITE_PATH = os.path.join(DIR, '../sites/')
REQUIREMENT = os.path.join(DIR, '../requirements/')


class BCDevice:
    def __init__(self, name, device):
        cflib.crtp.init_drivers()

        self.type = device['type']
        self.name = name
        self.link_uri = device['radio']
        self.cf = Crazyflie(rw_cache='./cache')
        self.bl = Bootloader(self.link_uri)

    def __str__(self):
        return 'test'

    def connect_sync(self, querystring=None):
        if querystring is None:
            uri = self.link_uri
        else:
            uri = self.link_uri + querystring

        self.cf.open_link(uri)

        ts = time.time()
        while not self.cf.is_connected():
            time.sleep(1.0 / 1000.0)
            delta = time.time() - ts
            if delta > self.CONNECT_TIMEOUT:
                return False
        return True


def get_ids(device):
    return device.name


def get_devices():
    devices = list()

    site = os.getenv('CRAZY_SITE')
    if site is None:
        raise Exception('No CRAZY_SITE env specified!')

    path = ""
    try:
        path = os.path.join(SITE_PATH, '%s.toml' % site)
        site_t = toml.load(open(path, 'r'))

        for name, device in site_t['device'].items():
            devices.append(BCDevice(name, device))
    except Exception:
        raise Exception('Failed to parse toml %s!' % path)

    return devices

def load_all_requirements():
    requirements = glob.glob(REQUIREMENT + '*.toml')
    req_dict = {}
    for requirement in requirements:
        req = toml.load(open(requirement))
        req_dict.update(req)

    return req_dict

def get_requirement(group: str, name: str):
    req = load_all_requirements()
    return req['requirement'][group][name]