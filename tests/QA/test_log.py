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
import time

from collections import defaultdict

from cflib.crazyflie.log import LogConfig
from conftest import BCDevice
from cflib.crazyflie.syncLogger import SyncLogger

class TestLogVariables:

    @pytest.mark.sanity
    @pytest.mark.exclude_decks('bcAI') #This fails with the ai deck sometimes. Flakyness.
    def test_log_async(self, connected_bc_dev: BCDevice):
        ''' Make sure we receive ~100 rows 1 second at 100Hz '''
        requirement = conftest.get_requirement('logging.basic')
        expected_rate = requirement['max_rate']  # Hz
        period_in_ms = int(1000 / expected_rate)
        duration = 5.0

        config = init_log_max_bytes(period_in_ms=period_in_ms)
        rows = 0

        def log_callback(ts, data, config):
            nonlocal rows
            rows += 1
            assert_variables_included(data, config.variables)

        connected_bc_dev.cf.log.add_config(config)
        config.data_received_cb.add_callback(log_callback)

        config.start()
        time.sleep(duration)
        config.stop()

        # Allow for 3% diff
        actual_rate = rows / duration
        assert_within_percentage(expected_rate, actual_rate, 3)

    def test_log_too_many_variables(self, connected_bc_dev: BCDevice):
        '''
        Make sure we get an AttributeError when adding more variables
        than logging.variables.max.
        '''
        def init_log_many_variables(name):
            config = LogConfig(name=name, period_in_ms=10)
            config.add_variable('stabilizer.roll', 'float')       # 1
            config.add_variable('stabilizer.pitch', 'float')      # 2
            config.add_variable('stabilizer.yaw', 'float')        # 3
            config.add_variable('stabilizer.thrust', 'uint16_t')  # 4

            config.add_variable('sys.canfly', 'uint8_t')          # 5
            config.add_variable('sys.isFlying', 'uint8_t')        # 6
            config.add_variable('sys.isTumbled', 'uint8_t')       # 7

            config.add_variable('radio.rssi', 'uint8_t')          # 8
            config.add_variable('radio.isConnected', 'uint8_t')   # 9

            config.add_variable('pm.batteryLevel', 'uint8_t')     # 10

            config.add_variable('health.motorPass', 'uint8_t')    # 11
            config.add_variable('health.batteryPass', 'uint8_t')  # 12

            return config

        requirement = conftest.get_requirement('logging.variables')
        configs = []
        for i in range(int(requirement['max'] / 12) + 1):
            configs.append(init_log_many_variables('ManyVariables_%d' % i))

        for config in configs:
            connected_bc_dev.cf.log.add_config(config)

        with pytest.raises(AttributeError):
            for config in configs:
                config.start()

    def test_log_too_many_blocks(self, connected_bc_dev: BCDevice):
        '''
        Make sure we get an AttributeError when adding more blocks
        than logging.blocks.max.
        '''
        requirement = conftest.get_requirement('logging.blocks')
        configs = []
        for i in range(requirement['max'] + 1):
            configs.append(init_log_max_bytes('MaxGroup_%d' % i))

        for config in configs:
            connected_bc_dev.cf.log.add_config(config)

        with pytest.raises(AttributeError):
            for config in configs:
                config.start()

    def test_log_too_much_per_block(self, connected_bc_dev: BCDevice):
        '''
        Make sure we get an AttributeError when adding more bytes
        than logging.blocks.max_payload to a LogConfig.
        '''
        config = init_log_max_bytes()

        # Adding one byte brings us to 27 bytes, and 26 (LogConfig.MAX_LEN) is max.
        config.add_variable('radio.rssi', 'uint8_t')

        with pytest.raises(AttributeError):
            connected_bc_dev.cf.log.add_config(config)

    @pytest.mark.sanity
    @pytest.mark.exclude_decks('bcDWM1000','bcFlow', 'bcFlow2', 'lighthouse4')
    def test_log_stress(self, connected_bc_dev: BCDevice):
        '''
        Make sure we can receive all packets requested when having an effective
        rate of logging.rate packets/s.
        '''
        requirement = conftest.get_requirement('logging.rate')

        configs = []
        duration = 10.0
        period_in_ms = 10
        expected_rate_per_block = 1000 / period_in_ms  # Hz
        expected_total_rate = requirement['limit_low']  # Hz
        nr_of_log_blocks = int(expected_total_rate / expected_rate_per_block)
        for i in range(nr_of_log_blocks):
            configs.append(init_log_max_bytes(f'MaxGroup_{i}', period_in_ms=period_in_ms))

        packets = defaultdict(lambda: 0)

        def stress_cb(ts, data, config):
            packets[config.name] += 1

        for config in configs:
            connected_bc_dev.cf.log.add_config(config)
            config.data_received_cb.add_callback(stress_cb)
            config.start()

        time.sleep(duration)

        for config in configs:
            config.stop()
            # Check the number of packets we got per config, allow for 3% margin.
            actual_rate_per_block = packets[config.name] / duration
            assert_within_percentage(expected_rate_per_block, actual_rate_per_block, 3)

        actual_total_rate = sum(packets.values()) / duration
        assert_within_percentage(expected_total_rate, actual_total_rate, 3)

    @pytest.mark.exclude_decks('bcAI')
    def test_log_sync(self, connected_bc_dev: BCDevice):
        ''' Make sure logging synchronous works '''
        requirement = conftest.get_requirement('logging.basic')
        config = init_log_max_bytes()

        with SyncLogger(connected_bc_dev.sync_cf, config) as logger:
            for rows, (ts, data, config) in enumerate(logger):
                assert_variables_included(data, config.variables)
                if rows >= requirement['max_rate']:
                    break


def init_log_max_bytes(name: str='MaxGroup', period_in_ms: int=10) -> LogConfig:
    ''' 7 variables * MAX_GROUPS (16) = 112 which is < MAX_VARIABLES (128) '''
    config = LogConfig(name=name, period_in_ms=period_in_ms)
    config.add_variable('stabilizer.roll', 'float')       # 04 bytes
    config.add_variable('stabilizer.pitch', 'float')      # 08 bytes
    config.add_variable('stabilizer.yaw', 'float')        # 12 bytes
    config.add_variable('stabilizer.thrust', 'uint16_t')  # 14 bytes

    config.add_variable('gyro.xVariance', 'float')        # 18 bytes
    config.add_variable('gyro.yVariance', 'float')        # 22 bytes
    config.add_variable('gyro.zVariance', 'float')        # 26 bytes

    return config


def assert_variables_included(data, variables):
    assert len(data) == len(variables)
    for v in variables:
        assert v.name in data


def assert_within_percentage(expected: float, actual: float, max_diff_percent: float):
    max_diff = expected * max_diff_percent
    assert actual >= expected - max_diff
    assert actual <= expected + max_diff
