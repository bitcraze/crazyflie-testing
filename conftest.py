from collections import namedtuple
from enum import Enum
import string
import pytest
import binascii
import os
import time
import toml
import glob
import struct
import sys
import subprocess
import logging
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from threading import Event
from typing import Callable, List, NoReturn, Optional

import cflib
from cflib.bootloader import Bootloader, Cloader, Target
from cflib.crazyflie import Crazyflie
from cflib.crtp.crtpstack import CRTPPacket
from cflib.crtp.crtpstack import CRTPPort
from cflib.utils.power_switch import PowerSwitch

from management.arduino_power_manager import RigManager

DIR = os.path.dirname(os.path.realpath(__file__))
SITE_PATH = os.path.join(DIR, 'sites/')
REQUIREMENT = os.path.join(DIR, 'requirements/')
DEFAULT_SITE = 'single-cf'

USB_Power_Control = namedtuple('Port', ['hub', 'port'])

ALL_DECKS= ['bcLighthouse4', 'bcFlow2', 'bcMultiranger', 'bcUSD', 'bcAI', 'bcLoco']

logger = logging.getLogger(__name__)


def pytest_generate_tests(metafunc):
    has_decks = metafunc.definition.get_closest_marker('decks')
    has_properties = metafunc.definition.get_closest_marker('requirements')
    exclude_decks = metafunc.definition.get_closest_marker('exclude_decks')
    has_decks = has_decks.args if has_decks else []
    has_properties = has_properties.args if has_properties else []
    exclude_decks = exclude_decks.args if exclude_decks else []
    devices = get_devices(has_decks,has_properties, exclude_decks)
    for fixture in metafunc.fixturenames:
        if fixture == 'request':
            continue
        if devices:
            metafunc.parametrize(fixture, devices, indirect=(fixture==fixture) , ids=lambda d: d.name)
        else:
            print(f'No devices found for test {metafunc.definition.name}')
            metafunc.parametrize(fixture, [pytest.param(None, marks=pytest.mark.ignore(reason="No device for test"))]) #This is a bit overly complicated but pytest.skip will skip all tests in modul

def pytest_collection_modifyitems(config, items):

    selected = list(items)
    deselected = []
    for test_item in items:
        if test_item.get_closest_marker('ignore'):
            selected.remove(test_item)
            deselected.append(test_item)

    items[:] = selected
    config.hook.pytest_deselected(items=deselected)

def get_rig_manager():
    site = os.getenv('CRAZY_SITE') or DEFAULT_SITE
    print(f'Using site {site}')
    if site is None:
        raise Exception('No CRAZY_SITE env specified!')
    path = os.path.join(SITE_PATH, '%s.toml' % site)
    site_t = toml.load(open(path, 'r'))
    try:
        addr = site_t['rig_management']
        return RigManager(addr)
    except KeyError:
        print("No rig manager for site")
        return None


class USB_Power_Control_Action(str, Enum):
    ON     = 'on'
    OFF    = 'off'
    TOGGLE = 'toggle'
    RESET  = 'reset'


class BCDevice:
    CONNECT_TIMEOUT = 10  # seconds

    def __init__(self, name, device):
        cflib.crtp.init_drivers()

        self.name = name
        self.link_uri = device['radio']

        self.usb_power_control = self._parse_usb_power_control(device)
        self.power_manager = None
        self.boot_time = 0.5
        self.sync_cf = None
        try:
            self.bl_link_uri = device['bootloader_radio']
        except KeyError:
            self.bl_link_uri = None
            pass

        self.decks = []
        self.properties = []
        if 'platform' in device:
            self.platform = device['platform']
        else :
            self.platform = "cf2"
        if 'decks' in device:
            if all(deck in ALL_DECKS for deck in device['decks']):
                self.decks = device['decks']
            else:
                raise Exception(f'Invalid decks in deck list of {self.name}: {device["decks"]}')
        if 'bcAI' in self.decks:
            self.boot_time = 5.5
        if 'properties' in device:
            self.properties = device['properties']
        if 'rig_management_addr' in device:
            self.power_manager = device['rig_management_addr']


    def start(self):
        self.cf = Crazyflie(rw_cache='./cache')

        self.cf.console.receivedChar.add_callback(_console_cb)

        self.bl = Bootloader(self.link_uri)

    def __str__(self):
        string = '{} @ {}'.format(self.name, self.link_uri)
        if self.usb_power_control is not None:
            hub, port = self.usb_power_control.hub, self.usb_power_control.port
            string += f' USB pwr-ctrl: [{hub}, {port}]'
        return string

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

    def power_cycle(self, rig_manager:RigManager=None):
        if self.power_manager is not None and rig_manager is not None:
           rig_manager.restart(self.power_manager)

    def goto_bootloader(self, rig_manager: RigManager=None):
        self.bl.close()
        self.bl = Bootloader(self.link_uri)
        if self.power_manager is not None and rig_manager is not None:
            print('Resetting to bootloader')
            rig_manager.bootloader(self.power_manager)
            time.sleep(2)


    def flash(self, path: str, progress_cb: Optional[Callable[[str, int], NoReturn]] = None, rig_manager:RigManager = None) -> bool:
        try:
            if path.name.endswith(".bin"):
                targets = [Target('cf2', 'stm32', 'fw', [], [])]
            else:
                targets = []
            try:
                self.power_cycle(rig_manager)
                self.bl.flash_full(cf=self.cf, filename=path, progress_cb=progress_cb, targets=targets,
                    enable_console_log=True, warm=True)
            except Exception as e:
                print(f'Failed to flash {path} to {self.name}. Resetting to bootloader and trying again')
                self.goto_bootloader(rig_manager)
                self.bl.flash_full(cf=self.cf, filename=path, progress_cb=progress_cb, targets=targets,
                    enable_console_log=True, warm=False)
                self.power_cycle(rig_manager)
        finally:
            self.bl.close()
            self.bl = Bootloader(self.link_uri)

    def connect_sync(self, querystring=None):
        if querystring is None:
            uri = self.link_uri
        else:
            uri = self.link_uri + querystring

        is_self_test_pass = False
        is_connected = self._wait_for_full_connection(self.cf, uri, self.CONNECT_TIMEOUT)
        if is_connected:
            is_self_test_pass = _verify_cf_self_test_pass(self.cf, uri)
        else:
            print(f'Failed to connect to Crazyflie at {uri}')

        return is_connected and is_self_test_pass

    def set_usb_power(self, action: USB_Power_Control_Action) -> bool:
        if self.usb_power_control is None:
            return False

        hub, port = self.usb_power_control.hub, self.usb_power_control.port
        cmd = f'uhubctl -l {hub} -p {port} -a {action}'

        print(f'> {cmd}')
        pipe = subprocess.Popen(
            cmd.split(' '),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        out = pipe.stdout.read()
        err = pipe.stderr.read()
        returncode = pipe.wait()

        if out:
            print(out.decode('utf-8'))
        if err:
            print(f'Error: {err.decode("utf-8")}')

        if returncode != 0:
            raise subprocess.CalledProcessError(
                returncode, cmd.split(' '), output=out, stderr=err
            )

        return True

    def _parse_usb_power_control(self, device: dict) -> USB_Power_Control:
        usb_power_control = device.get('usb_power_control')
        if usb_power_control is None:
            return None

        hub, port = usb_power_control.split(' ')
        return USB_Power_Control(hub, port)

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

@pytest.fixture
def connected_bc_dev(request):
    ''' This code will run before (and after) a test '''
    bcDev = request.param
    bcDev.start() #Create the crazyflie object
    with ValidatedSyncCrazyflie(bcDev.link_uri, cf=bcDev.cf) as cf:
        bcDev.sync_cf = cf  # Update the cf in the BCDevice object
        logger.info(f'Starting test with device {bcDev.name} @ {bcDev.link_uri}')
        yield bcDev  # code after this point will run as teardown after test
        bcDev.cf.close_link()
    logger.info(f'Finished test with device {bcDev.name} @ {bcDev.link_uri}')

@pytest.fixture
def unconnected_bc_dev(request):
    device = request.param
    device.start()
    yield device  # code after this point will run as teardown after test
    device.cf.close_link()

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

    return address


def get_devices(has_decks: List[str]=[], has_properties: List[str]=[], exclude_decks= []) -> List[BCDevice]:
    devices = list()

    site = os.getenv('CRAZY_SITE') or DEFAULT_SITE
    devicenames = os.getenv('CRAZY_DEVICE')
    print(f'Using site {site}')
    if site is None:
        raise Exception('No CRAZY_SITE env specified!')
    if devicenames is not None and devicenames != '':
        devicenames = devicenames.split(',')
    path = ""
    try:
        path = os.path.join(SITE_PATH, '%s.toml' % site)
        site_t = toml.load(open(path, 'r'))

        for name, device in site_t['device'].items():
            dev = BCDevice(name, device)
            conditions = [
                (not devicenames or name in devicenames),
                all(deck in dev.decks for deck in has_decks),
                all(prop in dev.properties for prop in has_properties),
                all(deck not in dev.decks for deck in exclude_decks)
            ]
            if all(conditions):
                    print(f'Adding device {name} to test')
                    devices.append(dev)
    except Exception:
        raise Exception(f'Failed to parse toml {path}!')
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


def _verify_cf_self_test_pass(cf: Crazyflie, uri: str) -> bool:
    is_self_test_pass = bool(int(cf.param.get_value('system.selftestPassed')))

    if not is_self_test_pass:
        print(f'The Crazyflie did not pass self tests ({uri})')

        # Trigger a dump of assert info
        cf.param.set_value('system.assertInfo', 1)

        # Wait a bit for all console logs to arrive
        time.sleep(0.5)

        # Console logs are captured and printed by default, but are only displayed when a test case fails.

    return is_self_test_pass

def _console_cb(msg):
    # Prints are only displayed if a dest fails
    print(f'Console: {msg}')


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


class ValidatedSyncCrazyflie(SyncCrazyflie):
    """Use this class instead of SyncCrazyflie in tests. This class does extra checks when connecting to make sure
    the CF is OK.
    """
    def __init__(self, link_uri: str, cf=None):
        super().__init__(link_uri, cf=cf)

    def __enter__(self):
        self.open_link()

        self.cf.console.receivedChar.add_callback(_console_cb)

        is_self_test_pass = _verify_cf_self_test_pass(self.cf, self._link_uri)
        assert is_self_test_pass

        return self
    def __exit__(self, exc_type, exc_val, exc_tb):
        logger.info('Exciting')
        self.cf.console.receivedChar.remove_callback(_console_cb)
        super().__exit__(exc_type, exc_val, exc_tb)
