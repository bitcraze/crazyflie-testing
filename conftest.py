from collections import namedtuple
from enum import Enum
import pytest
import os
import time
import toml
import glob
import sys
import subprocess
import logging
import threading
from typing import List
import tempfile
from pathlib import Path

from cflib._rust import Crazyflie, FileTocCache

from management.arduino_power_manager import RigManager

DIR = os.path.dirname(os.path.realpath(__file__))
SITE_PATH = os.path.join(DIR, 'sites/')
REQUIREMENT = os.path.join(DIR, 'requirements/')
DEFAULT_SITE = 'single-cf'

USB_Power_Control = namedtuple('Port', ['hub', 'port'])

ALL_DECKS= ['bcLighthouse4', 'bcFlow2', 'bcMultiranger', 'bcUSD', 'bcAI', 'bcLoco']

logger = logging.getLogger(__name__)

# Initialize TOC cache using temp directory (same pattern as lib examples)
CACHE_DIR = str(Path(tempfile.gettempdir()) / "crazyflie_toc_cache")
TOC_CACHE = FileTocCache(CACHE_DIR)


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


class BootloaderStub:
    """Stub for bootloader functionality - not yet available in Rust backend"""

    def __init__(self, link_uri):
        self.link_uri = link_uri

    def start_bootloader(self, warm_boot=False):
        """Start the bootloader"""
        raise NotImplementedError("Bootloader support not yet available in Rust backend")

    def reset_to_firmware(self):
        """Reset from bootloader back to firmware"""
        raise NotImplementedError("Bootloader support not yet available in Rust backend")

    def close(self):
        """Close bootloader connection"""
        raise NotImplementedError("Bootloader support not yet available in Rust backend")


class BCDevice:
    CONNECT_TIMEOUT = 10  # seconds

    def __init__(self, name, device):
        self.name = name
        self.link_uri = device['radio']

        self.usb_power_control = self._parse_usb_power_control(device)
        self.power_manager = None
        self.boot_time = 0.5
        self.sync_cf = None
        self._console_thread = None
        self._console_running = False

        # Bootloader support (stub until Rust backend implements it)
        try:
            self.bl_link_uri = device['bootloader_radio']
        except KeyError:
            self.bl_link_uri = None

        # Create bootloader stub
        self.bl = BootloaderStub(self.bl_link_uri if self.bl_link_uri else self.link_uri)

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

    def __str__(self):
        string = '{} @ {}'.format(self.name, self.link_uri)
        if self.usb_power_control is not None:
            hub, port = self.usb_power_control.hub, self.usb_power_control.port
            string += f' USB pwr-ctrl: [{hub}, {port}]'
        return string

    def _start_console_polling(self):
        """Start background thread to poll console output"""
        def console_poll_loop():
            while self._console_running:
                try:
                    if self.cf is not None:
                        console = self.cf.console()
                        lines = console.get_lines()
                        for line in lines:
                            print(f'Console: {line}')
                except Exception:
                    pass  # Ignore errors during shutdown
                time.sleep(0.1)  # Poll every 100ms

        self._console_running = True
        self._console_thread = threading.Thread(target=console_poll_loop, daemon=True)
        self._console_thread.start()

    def _stop_console_polling(self):
        """Stop console polling thread"""
        self._console_running = False
        if self._console_thread:
            self._console_thread.join(timeout=1.0)
            self._console_thread = None

    def disconnect(self):
        """Disconnect from the Crazyflie and clean up resources"""
        if self.cf is not None:
            self._stop_console_polling()
            self.cf.disconnect()
            self.cf = None

    def firmware_up(self) -> bool:
        """Check if firmware is running"""
        raise NotImplementedError("firmware_up() not yet migrated to Rust backend")

    def reboot(self):
        """Reboot the Crazyflie"""
        raise NotImplementedError("reboot() not yet migrated to Rust backend")

    def power_cycle(self, rig_manager:RigManager|None=None):
        if self.power_manager is not None and rig_manager is not None:
           rig_manager.restart(self.power_manager)

    def connect_sync(self, querystring=None):
        """Connect to the Crazyflie using new synchronous API"""
        if querystring is None:
            uri = self.link_uri
        else:
            uri = self.link_uri + querystring

        try:
            # Single synchronous call creates AND connects
            self.cf = Crazyflie.connect_from_uri(uri, toc_cache=TOC_CACHE)

            # Start console polling thread
            self._start_console_polling()

            # Verify self-test passed
            is_self_test_pass = _verify_cf_self_test_pass(self.cf, uri)

            if not is_self_test_pass:
                self._stop_console_polling()
                self.cf.disconnect()
                self.cf = None
                return False

            return True

        except Exception as e:
            print(f'Failed to connect to Crazyflie at {uri}: {e}')
            self.cf = None
            return False

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

        out = pipe.stdout.read() if pipe.stdout else None
        err = pipe.stderr.read() if pipe.stderr else None
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

    def _parse_usb_power_control(self, device: dict) -> USB_Power_Control| None:
        usb_power_control = device.get('usb_power_control')
        if usb_power_control is None:
            return None

        hub, port = usb_power_control.split(' ')
        return USB_Power_Control(hub, port)

@pytest.fixture
def connected_bc_dev(request):
    """Provides a connected BCDevice for tests"""
    bcDev = request.param

    # Connect to device (this also starts console polling)
    logger.info(f'Connecting to device {bcDev.name} @ {bcDev.link_uri}')
    if not bcDev.connect_sync():
        pytest.fail(f'Failed to connect to {bcDev.name}')

    # Set sync_cf for backwards compatibility with 2 tests that use it
    # In Rust backend, cf is already synchronous, so just point to same object
    bcDev.sync_cf = bcDev.cf

    logger.info(f'Starting test with device {bcDev.name} @ {bcDev.link_uri}')

    try:
        yield bcDev
    finally:
        # Cleanup
        if bcDev.cf is not None:
            logger.info(f'Disconnecting from device {bcDev.name}')
            bcDev._stop_console_polling()
            bcDev.cf.disconnect()
            bcDev.cf = None

        logger.info(f'Finished test with device {bcDev.name} @ {bcDev.link_uri}')

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
    # Get param subsystem (returns Param object)
    param = cf.param()

    # Use .get() instead of .get_value()
    is_self_test_pass = bool(int(param.get('system.selftestPassed')))

    if not is_self_test_pass:
        print(f'The Crazyflie did not pass self tests ({uri})')

        # Trigger a dump of assert info
        # Use .set() instead of .set_value()
        param.set('system.assertInfo', 1)

        # Wait a bit for all console logs to arrive
        time.sleep(0.5)

        # Console logs are captured and printed by default, but are only displayed when a test case fails.

    return is_self_test_pass


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
