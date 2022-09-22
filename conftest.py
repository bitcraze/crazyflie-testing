import string
import pytest
import binascii
import os
import time
import toml
import glob
import struct
import sys

from threading import Event
from typing import Callable
from typing import List
from typing import NoReturn
from typing import Optional

import cflib
from cflib.bootloader import Bootloader, Cloader, Target
from cflib.crazyflie import Crazyflie
from cflib.crtp.crtpstack import CRTPPacket
from cflib.crtp.crtpstack import CRTPPort
from cflib.utils.power_switch import PowerSwitch

DIR = os.path.dirname(os.path.realpath(__file__))
SITE_PATH = os.path.join(DIR, 'sites/')
REQUIREMENT = os.path.join(DIR, 'requirements/')


class BCDevice:
    CONNECT_TIMEOUT = 10  # seconds

    def __init__(self, name, device):
        cflib.crtp.init_drivers()

        self.name = name
        self.link_uri = device['radio']
        try:
            self.bl_link_uri = device['bootloader_radio']
        except KeyError:
            self.bl_link_uri = None
            pass

        self.decks = []

        if 'decks' in device:
            self.decks = device['decks']

        self.cf = Crazyflie(rw_cache='./cache')

        self.console_log = ''
        self.cf.console.receivedChar.add_callback(self._console_cb)

        self.bl = Bootloader(self.link_uri)

    def __str__(self):
        return '{} @ {}'.format(self.name, self.link_uri)

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

    def reboot(self):
        switch = PowerSwitch(self.link_uri)
        switch.stm_power_cycle()

    def recover(self):
        if self.bl_link_uri is None:
            return False

        cloader = Cloader(None)
        cloader.link = cflib.crtp.get_link_driver(self.bl_link_uri)
        if cloader.link is None:
            return False

        status = cloader.reset_to_firmware(0xFE)  # nrf target_id
        cloader.link.close()

        return status

    def flash(self, path: str, progress_cb: Optional[Callable[[str, int], NoReturn]] = None) -> bool:
        try:
            if path.name.endswith(".bin"):
                targets = [Target('cf2', 'stm32', 'fw')]
            else:
                targets = []

            self.bl.flash_full(cf=self.cf, filename=path, progress_cb=progress_cb, targets=targets)
        except Exception as e:
            raise e
        finally:
            self.bl.close()

    def connect_sync(self, querystring=None):
        self.cf.close_link()

        if querystring is None:
            uri = self.link_uri
        else:
            uri = self.link_uri + querystring

        is_success = self._wait_for_full_connection(self.cf, uri, self.CONNECT_TIMEOUT)
        if is_success:
            is_success = self._verify_cf_self_test_pass(self.cf)

            if not is_success:
                # Wait a bit for all console logs to arrive
                time.sleep(0.5)
                print(f'The Crazyflie did not pass self tests ({uri})')
                print('--- Begin dump console log')
                print(self.console_log)
                print('--- End dump console log')
        else:
            print(f'Failed to connect to Crazyflie at {uri}')

        return is_success

    def _wait_for_full_connection(self, cf: Crazyflie, uri: string, timeout: float) -> bool:
        connection_event = Event()
        def connection_cb(uri):
            nonlocal connection_event
            connection_event.set()

        cf.fully_connected.add_callback(connection_cb)
        cf.open_link(uri)
        is_connection_success = connection_event.wait(timeout=timeout)
        cf.fully_connected.remove_callback(connection_cb)

        return is_connection_success

    def _verify_cf_self_test_pass(self, cf: Crazyflie) -> bool:
        selftest_passed = cf.param.get_value('system.selftestPassed')
        result = bool(int(selftest_passed))
        return result

    def _console_cb(self, msg):
        self.console_log += msg

class DeviceFixture:
    def __init__(self, dev: BCDevice):
        self._device = dev

    @property
    def device(self) -> BCDevice:
        return self._device

    @property
    def kalman_active(self) -> bool:
        kalman_decks = ['bcLighthouse4', 'bcFlow', 'bcFlow2', 'bcDWM1000']
        if self._device.decks:
            return all(deck in kalman_decks for deck in self._device.decks)
        else:
            return False


@pytest.fixture
def test_setup(request):
    ''' This code will run before (and after) a test '''
    fix = DeviceFixture(request.param)
    yield fix  # code after this point will run as teardown after test
    fix.device.cf.close_link()


def get_bl_address(dev: BCDevice) -> str:
    '''
    Send the BOOTLOADER_CMD_RESET_INIT command to the NRF firmware
    and receive the bootloader radio address in the response
    '''
    address = None
    link = cflib.crtp.get_link_driver(dev.link_uri)
    if link is None:
        return None

    # 0xFF => BOOTLOADER CMD
    # 0xFE => To the NRF firmware
    # 0xFF => BOOTLOADER_CMD_RESET_INIT (to get bl address)
    pk = CRTPPacket(0xFF, [0xFE, 0xFF])
    link.send_packet(pk)

    timeout = 5  # seconds
    ts = time.time()
    while time.time() - ts < timeout:
        pk = link.receive_packet(2)
        if pk is None:
            continue

        # Header 0xFF means port is 0xF ((header & 0xF0) >> 4)) and channel
        # is 0x3 (header & 0x03).
        if pk.port == 0xF and pk.channel == 0x3 and len(pk.data) > 3:
            # 0xFE is NRF target id, 0xFF is BOOTLOADER_CMD_RESET_INIT
            if struct.unpack('<BB', pk.data[0:2]) != (0xFE, 0xFF):
                continue
            address = 'B1' + binascii.hexlify(pk.data[2:6][::-1]).upper().decode('utf8')  # noqa
            break

    link.close()
    return address


def get_devices() -> List[BCDevice]:
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


def get_swarm() -> List[BCDevice]:
    '''
    Given a path to the Crazyswarm project source and path in the
    CRAZYSWARM_PATH environment variable and a path to a YAML file defining
    a swarm in CRAZYSWARM_YAML return a list of BCDevice.
    '''
    devices = list()

    try:
        crazyswarm_path = os.environ['CRAZYSWARM_PATH']
        sys.path.append(os.path.join(
            crazyswarm_path,
            'ros_ws/src/crazyswarm/scripts'
        ))
        sys.path.append(os.path.join(
            crazyswarm_path,
            'ros_ws/src/crazyflie_ros'
        ))
        from pycrazyswarm import Crazyswarm

        crazyflies_yaml = os.path.join(
            os.path.dirname(os.path.realpath(__file__)),
            'swarms',
            os.environ['CRAZYSWARM_YAML']
        )
        cs = Crazyswarm(crazyflies_yaml=crazyflies_yaml)

        for cf in cs.allcfs.crazyflies:
            address = 'E7E7E7E7{:X}'.format(cf.id)

            # get URI from address using scan
            found = cflib.crtp.scan_interfaces(int(address, 16))
            if not found:
                raise Exception(f'No device found @ {address}!')

            dev = BCDevice(
                name=f'swarm-{cf.id}',
                device={
                    'radio': found[0][0],
                    'bootloader_radio': None,
                }
            )
            devices.append(dev)
    except KeyError as err:
        print('CRAZYSWARM_PATH or CRAZYSWARM_YAML not set', file=sys.stderr)
        raise err
    except ImportError as err:
        print('Failed to import pycrazyswarm', file=sys.stderr)
        raise err

    return devices


class Requirements(dict):
    _instance = None

    def __init__(self):
        raise RuntimeError('Call instance() instead')

    @classmethod
    def _read_requirements(cls):
        requirements = glob.glob(REQUIREMENT + '*.toml')
        for requirement in requirements:
            req = toml.load(open(requirement))
            for key, value in req.items():
                if type(value) == dict:
                    if key not in cls._instance:
                        cls._instance[key] = {}
                    for subkey, subvalue in value.items():
                        cls._instance[key][subkey] = subvalue
                else:
                    cls._instance[key] = value

    @classmethod
    def instance(cls):
        if cls._instance is None:
            cls._instance = cls.__new__(cls)
            cls._read_requirements()
        return cls._instance


def get_requirement(requirement: str):
    group, name = requirement.split('.')
    return Requirements.instance()['requirement'][group][name]
