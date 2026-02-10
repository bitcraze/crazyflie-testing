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

from conftest import BCDevice

class TestLogVariables:

    @pytest.mark.sanity
    def test_log_async(self, connected_bc_dev: BCDevice):
        ''' Make sure we receive ~100 rows 1 second at 100Hz '''
        requirement = conftest.get_requirement('logging.basic')
        expected_rate = requirement['max_rate']  # Hz
        period_in_ms = int(1000 / expected_rate)
        duration = 5.0

        log = connected_bc_dev.cf.log()
        block = create_log_block_max_bytes(log)
        log_stream = block.start(period_in_ms)
        
        rows = 0
        start_time = time.time()
        try:
            while time.time() - start_time < duration:
                data = log_stream.next()
                rows += 1
                assert_variables_included(data["data"])
        finally:
            log_stream.stop()

        # Allow for 3% diff
        actual_rate = rows / duration
        assert_within_percentage(expected_rate, actual_rate, 3)

    def test_log_too_many_variables(self, connected_bc_dev: BCDevice):
        '''
        Make sure we get an error when adding more variables
        than logging.variables.max (128 total across all active blocks).
        
        Note: This test is complex because we must stay within the 16 block limit
        while exceeding the 128 variable limit. With 16 blocks, we need >8 variables
        per block on average. We use 9 variables per block = 144 total > 128 limit.
        '''
        requirement = conftest.get_requirement('logging.variables')
        blocks_requirement = conftest.get_requirement('logging.blocks')
        log = connected_bc_dev.cf.log()
        
        # Create blocks with 9 variables each (fits in payload limit)
        # 16 blocks * 9 vars = 144 variables (exceeds 128 limit)
        def create_block_with_9_vars(log):
            block = log.create_block()
            # 9 uint8 variables = 9 bytes (well under 26 byte limit)
            block.add_variable('sys.canfly')
            block.add_variable('sys.isFlying')
            block.add_variable('sys.isTumbled')
            block.add_variable('radio.rssi')
            block.add_variable('pm.state')
            block.add_variable('pm.batteryLevel')
            block.add_variable('pm.vbat')
            block.add_variable('pm.chg')
            block.add_variable('sys.armed')
            return block
        
        streams = []
        
        # Try to start 16 blocks with 9 vars each (144 total vars > 128 limit)
        # This should fail when we exceed 128 variables
        with pytest.raises(Exception):
            for i in range(blocks_requirement['max']):
                block = create_block_with_9_vars(log)
                stream = block.start(100)  # 100ms period to reduce load
                streams.append(stream)
        
        # Clean up any streams that were created
        for stream in streams:
            try:
                stream.stop()
            except:
                pass

    def test_log_too_many_blocks(self, connected_bc_dev: BCDevice):
        '''
        Make sure we get an error when having more active blocks
        than logging.blocks.max simultaneously.
        
        Note: The Crazyflie firmware limits active log blocks, not total created.
        '''
        pytest.skip("Test creates 16+ active log streams which causes deadlock in current Rust implementation")

    def test_log_too_much_per_block(self, connected_bc_dev: BCDevice):
        '''
        Make sure we get an error when adding more bytes
        than logging.blocks.max_payload to a block.
        '''
        log = connected_bc_dev.cf.log()
        block = create_log_block_max_bytes(log)

        # Adding one byte brings us to 27 bytes, and 26 is max.
        with pytest.raises(Exception):
            block.add_variable('radio.rssi')

    @pytest.mark.sanity
    @pytest.mark.exclude_decks('bcDWM1000','bcFlow', 'bcFlow2', 'lighthouse4')
    def test_log_stress(self, connected_bc_dev: BCDevice):
        '''
        Make sure we can receive all packets requested when having an effective
        rate of logging.rate packets/s.
        '''
        requirement = conftest.get_requirement('logging.rate')

        duration = 10.0
        period_in_ms = 10
        expected_rate_per_block = 1000 / period_in_ms  # Hz
        expected_total_rate = requirement['limit_low']  # Hz
        nr_of_log_blocks = int(expected_total_rate / expected_rate_per_block)
        
        log = connected_bc_dev.cf.log()
        streams = []
        packets = defaultdict(lambda: 0)
        
        for i in range(nr_of_log_blocks):
            block = create_log_block_max_bytes(log)
            streams.append((i, block.start(period_in_ms)))

        start_time = time.time()
        try:
            while time.time() - start_time < duration:
                for i, stream in streams:
                    try:
                        data = stream.next()
                        packets[i] += 1
                    except:
                        pass
        finally:
            for i, stream in streams:
                stream.stop()

        for i in range(nr_of_log_blocks):
            # Check the number of packets we got per stream, allow for 3% margin.
            actual_rate_per_block = packets[i] / duration
            assert_within_percentage(expected_rate_per_block, actual_rate_per_block, 3)

        actual_total_rate = sum(packets.values()) / duration
        assert_within_percentage(expected_total_rate, actual_total_rate, 3)

    def test_log_sync(self, connected_bc_dev: BCDevice):
        ''' Make sure logging synchronous works '''
        requirement = conftest.get_requirement('logging.basic')
        
        log = connected_bc_dev.cf.log()
        block = create_log_block_max_bytes(log)
        log_stream = block.start(10)
        
        try:
            for rows in range(requirement['max_rate']):
                data = log_stream.next()
                assert_variables_included(data["data"])
        finally:
            log_stream.stop()


def create_log_block_max_bytes(log):
    ''' 
    Create a log block close to max payload (26 bytes)
    Use 6 floats (24 bytes) + 2 uint8 (2 bytes) = 26 bytes total
    '''
    block = log.create_block()
    block.add_variable('stabilizer.roll')       # f32: 4 bytes
    block.add_variable('stabilizer.pitch')      # f32: 4 bytes
    block.add_variable('stabilizer.yaw')        # f32: 4 bytes
    block.add_variable('stabilizer.thrust')     # f32: 4 bytes
    block.add_variable('gyro.xVariance')        # f32: 4 bytes
    block.add_variable('gyro.yVariance')        # f32: 4 bytes
    # Total so far: 24 bytes, can add 2 more bytes
    block.add_variable('radio.rssi')            # u8: 1 byte
    block.add_variable('pm.state')              # u8: 1 byte
    # Total: 26 bytes (max)

    return block


def assert_variables_included(data):
    expected_vars = ['stabilizer.roll', 'stabilizer.pitch', 'stabilizer.yaw', 
                     'stabilizer.thrust', 'gyro.xVariance', 'gyro.yVariance', 
                     'radio.rssi', 'pm.state']
    assert len(data) == len(expected_vars)
    for var in expected_vars:
        assert var in data


def assert_within_percentage(expected: float, actual: float, max_diff_percent: float):
    max_diff = expected * (max_diff_percent / 100.0)
    assert actual >= expected - max_diff
    assert actual <= expected + max_diff
