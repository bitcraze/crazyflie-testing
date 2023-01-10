# Copyright (C) 2021 Bitcraze AB
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, in version 3.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.
import pytest
import conftest
import logging
import time
import random
from threading import Event

from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

logger = logging.getLogger(__name__)


#
# Using the indirect=True parameter when parametrizing a test allows to
# parametrize a test with a fixture receiving the values before passing them to
# a test. In this case it means a device in the array returned from
# get_devices() will be passed to test_setup() in conftest.py before being used
# as a parameter in the test methods.
#
@pytest.mark.parametrize(
    'test_setup',
    conftest.get_devices(),
    indirect=True,
    ids=lambda d: d.name
)
class TestParameters:
    def test_param_ronly(self, test_setup):
        with SyncCrazyflie(test_setup.device.link_uri) as scf:
            # Get a known (core) read-only parameter
            param = "deck.bcLighthouse4"
            element = scf.cf.param.toc.get_element_by_complete_name(param)
            assert element is not None

            # Make sure it is marked as read-only
            assert element.get_readable_access() == "RO"

            # Make sure we get an error if we try to set it
            with pytest.raises(AttributeError):
                scf.cf.param.set_value(param, 1)

    def test_param_extended_type(self, test_setup):
        with SyncCrazyflie(test_setup.device.link_uri) as scf:
            # Get a known persistent parameter
            param = "ring.effect"
            element = scf.cf.param.toc.get_element_by_complete_name(param)
            assert element is not None
            assert element.is_extended()
            assert element.is_persistent()

            # And a known non-persistent parameter
            param = "stabilizer.stop"
            element = scf.cf.param.toc.get_element_by_complete_name(param)
            print(element.is_persistent)
            assert element is not None
            assert not element.is_extended()
            assert not element.is_persistent()

    def test_param_persistent_store(self, test_setup):
        # Get a known persistent parameter
        param = "sound.effect"

        # Get a random valid value
        value = random.randint(8, 13)

        with SyncCrazyflie(test_setup.device.link_uri) as scf:
            # Set Value
            logger.info(f"Setting value {value} as {param}")
            scf.cf.param.set_value(param, value)

            got_callback = False

            def store_cb(name, success):
                nonlocal got_callback
                assert name == param
                assert success

                got_callback = True

            scf.cf.param.persistent_store(param, store_cb)
            tries = 5
            while not got_callback and tries > 0:
                time.sleep(1)
                tries -= 1
            assert got_callback

            test_setup.device.reboot()

        # Allow time to reboot
        time.sleep(5)

        with SyncCrazyflie(test_setup.device.link_uri) as scf:
            val = scf.cf.param.get_value(param)
            assert int(val) == value

    def test_param_persistent_clear(self, test_setup):
        with SyncCrazyflie(test_setup.device.link_uri) as scf:
            # Get a known persistent parameter
            param = "sound.effect"

            gotten_state = False

            def clear_cb(name, success):
                assert name == param
                assert success

            def state_cb_1(name, state):
                nonlocal gotten_state

                assert name == param

                assert state is not None
                assert isinstance(state.is_stored, bool)
                assert state.default_value == 0
                if state.is_stored:
                    scf.cf.param.persistent_clear(param, clear_cb)
                    assert state.stored_value is not None
                else:
                    assert state.stored_value is None

                gotten_state = True

            scf.cf.param.persistent_get_state(param, state_cb_1)
            tries = 5
            while not gotten_state and tries > 0:
                time.sleep(1)
                tries -= 1
            assert gotten_state

            # Allow time to reboot
            time.sleep(5)
            gotten_state = False

            def state_cb_2(name, state):
                nonlocal gotten_state

                assert name == param
                assert state is not None
                assert isinstance(state.is_stored, bool)
                assert not state.is_stored
                gotten_state = True

            scf.cf.param.persistent_get_state(param, state_cb_2)
            tries = 5
            while not gotten_state and tries > 0:
                time.sleep(1)
                tries -= 1
            assert gotten_state

    def test_param_persistent_get_state(self, test_setup):
        with SyncCrazyflie(test_setup.device.link_uri) as scf:
            # Get a known persistent parameter
            param = "sound.effect"

            gotten_state = False

            def state_cb(name, state):
                nonlocal gotten_state

                assert name == param
                assert state is not None
                logger.info(f"state: {state}")
                assert isinstance(state.is_stored, bool)
                assert state.default_value == 0
                if state.is_stored:
                    assert state.stored_value is not None
                else:
                    assert state.stored_value is None

                gotten_state = True

            scf.cf.param.persistent_get_state(param, state_cb)
            tries = 5
            while not gotten_state and tries > 0:
                time.sleep(1)
                tries -= 1
            assert gotten_state

            # Attempt to get state from non-persistent param,
            # make sure we get AttributeError.
            param = "stabilizer.stop"
            with pytest.raises(AttributeError):
                scf.cf.param.persistent_get_state(param, state_cb)


    def test_param_persistent_eeprom_stress(self, test_setup):
        """ Stress test the eeprom by setting and clearing persistent parameters. This will create holes in the eeprom
            memory which needs to be de-fragmented once it hits this limit, which should be between 250-300 persistent
            parameters.
            Clear after store to make sure we do not have too many stored parameters that fill up the memory and
            prevent further storage.
        """

        max_avg_sec_per_parameter = 0.5  # in sec
        max_sec_defrag = 3.0  # in sec

        def get_all_persistent_param_names(cf):
            persistent_params = []
            for group_name, params in cf.param.toc.toc.items():
                for param_name, element in params.items():
                    if element.is_persistent():
                        complete_name = group_name + "." + param_name
                        persistent_params.append(complete_name)

            return persistent_params

        def store_persistent(scf, complete_name: str) -> None:
            wait_for_callback_event = Event()

            def is_done_callback(complete_name, success):
                if not success:
                    dump_storage_stats(scf)

                assert success
                wait_for_callback_event.set()

            scf.cf.param.persistent_store(complete_name, callback=is_done_callback)
            assert wait_for_callback_event.wait(timeout=5)

        def clear_persistent(scf, complete_name: str) -> None:
            wait_for_callback_event = Event()

            def is_done_callback(complete_name, success):
                assert success
                wait_for_callback_event.set()

            scf.cf.param.persistent_clear(complete_name, callback=is_done_callback)
            assert wait_for_callback_event.wait(timeout=5)

        def is_persistent_stored(scf, complete_name: str) -> bool:
            wait_for_callback_event = Event()
            result = False

            def is_done_callback(complete_name, state):
                nonlocal result
                result = state.is_stored
                wait_for_callback_event.set()

            scf.cf.param.persistent_get_state(complete_name, callback=is_done_callback)
            assert wait_for_callback_event.wait(timeout=5)

            return result

        def dump_storage_stats(scf):
            print('Dumping storage stats...')
            scf.cf.param.set_value('system.storageStats', 1)
            # Wait for logs to arrive
            time.sleep(1)

        with SyncCrazyflie(test_setup.device.link_uri) as scf:
            # Enable logging to print stats if dump_storage_stats() is called
            scf.cf.console.receivedChar.add_callback(lambda msg: print(f'Console log: {msg}'))

            # Get the names of all parameters that can be persisted
            persistent_params = get_all_persistent_param_names(scf.cf)
            assert persistent_params is not None

            # Clear all persistent parameters that are set
            for param_name in persistent_params:
                if is_persistent_stored(scf, param_name):
                    clear_persistent(scf, param_name)

            total_time = []

            count = 300
            while count > 0:
                for param_name in persistent_params:
                    start_time = time.time()
                    # Store and clear to make sure we do not fill the memory
                    store_persistent(scf, param_name)
                    assert is_persistent_stored(scf, param_name)
                    clear_persistent(scf, param_name)
                    assert not is_persistent_stored(scf, param_name)
                    total_time.append(time.time() - start_time)

                    count -= 1
                    if count < 0:
                        break

            # Verify average time
            average_time = sum(total_time[1:]) / len(total_time[1:])
            logger.info(f"Average time {average_time}")
            assert average_time < max_avg_sec_per_parameter

            # Verify maximum time, probably at de-frag
            max_time = max(total_time[1:])
            logger.info(f"Max time {max_time}")
            assert max_time < max_sec_defrag

    def test_param_set_raw(self, test_setup):
        param = "ring.effect"
        value = 13  # Gravity effect
        updated = False

        def param_raw_cb(name: str, val: str):
            nonlocal param
            nonlocal value
            nonlocal updated

            assert name == param
            assert value == int(val)
            updated = True

        with SyncCrazyflie(test_setup.device.link_uri) as scf:
            [group, name] = param.split(".")

            scf.wait_for_params()

            # 0x08 = UINT_8,
            scf.cf.param.add_update_callback(group=group, name=name, cb=param_raw_cb)
            scf.cf.param.set_value_raw(param, 0x08, value)
            scf.cf.param.request_param_update(param)

            tries = 5
            while not updated and tries > 0:
                time.sleep(1)
                tries -= 1

            assert updated

    def test_param_set(self, test_setup):
        param = "stabilizer.estimator"

        def param_cb(name: str, value: str):
            nonlocal expected
            nonlocal param

            assert name == param
            assert expected.pop(0) == int(value)

        with SyncCrazyflie(test_setup.device.link_uri) as scf:
            [group, name] = param.split(".")

            initial = scf.cf.param.get_value(param)
            assert initial is not None

            expected = [2, 1, 2, 1, int(initial)]

            scf.cf.param.add_update_callback(group=group, name=name, cb=param_cb)

            scf.cf.param.set_value(param, 2)
            scf.cf.param.set_value(param, 1)
            scf.cf.param.set_value(param, 2)
            scf.cf.param.set_value(param, 1)

            scf.cf.param.set_value(param, int(initial))

            timeout = 5  # seconds
            time.sleep(timeout)

            assert len(expected) == 0
