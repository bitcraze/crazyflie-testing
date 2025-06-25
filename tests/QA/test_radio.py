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
import time
import struct

import cflib.crtp
from cflib.crtp.crtpstack import CRTPPacket
from cflib.crtp.crtpstack import CRTPPort
from conftest import BCDevice
from cflib.utils.callbacks import Syncer

import conftest
import logging

logger = logging.getLogger(__name__)

@pytest.mark.sanity
class TestRadio:
    def test_latency(self, connected_bc_dev: BCDevice):
        requirement = conftest.get_requirement('radio.latency')
        assert(latency(connected_bc_dev) < requirement['limit_high_ms'])

    @pytest.mark.requirements("syslink_flowctrl")
    def test_bandwidth_small_packets(self, unconnected_bc_dev: BCDevice):
        requirement = conftest.get_requirement('radio.bwsmall')
        assert(bandwidth(unconnected_bc_dev.link_uri, requirement['packet_size']) > requirement['limit_low'])

    @pytest.mark.requirements("syslink_flowctrl")
    def test_bandwidth_big_packets(self, unconnected_bc_dev: BCDevice):
        requirement = conftest.get_requirement('radio.bwbig')
        assert(bandwidth(unconnected_bc_dev.link_uri, requirement['packet_size']) > requirement['limit_low'])

    @pytest.mark.requirements("syslink_flowctrl")
    def test_reliability(self, unconnected_bc_dev: BCDevice):
        requirement = conftest.get_requirement('radio.reliability')
        # The bandwidth function will assert if there is any packet loss
        ping(unconnected_bc_dev.link_uri, requirement['packet_size'], requirement['limit_low'])


def latency(connected_bc_dev: conftest.BCDevice, timeout=10):
    """
    Retrieve the latency to a Crazyflie.

    Args:
        uri (str): The URI of the Crazyflie.
        cf (Crazyflie): The Crazyflie instance to retrieve the latency from.
        timeout (float): Maximum time to wait for latency updates.

    Returns:
        float: The latency value received.

    Raises:
        TimeoutError: If the timeout is reached during latency retrieval.
    """

    time.sleep(10)  # wait 10 seconds for the p95 latency to converge

    syncer = Syncer()

    def on_latency_update(latency):
        syncer.success_cb(latency)

    # Add the callback
    connected_bc_dev.cf.link_statistics.latency_updated.add_callback(on_latency_update)

    try:
        # Wait for latency update
        success = syncer._event.wait(timeout)
        if not success:
            raise TimeoutError("Timed out waiting for a latency update.")
        latency = syncer.success_args[0]
        logger.info('latency: {}'.format(latency))
        return latency
    finally:
        connected_bc_dev.cf.link_statistics.latency_updated.remove_callback(on_latency_update)


def build_data(i, packet_size):
    repeats = packet_size // 4
    remainder = packet_size % 4

    data = struct.pack('<' + 'I' * repeats, *[i] * repeats)

    if remainder:  #Pad with 0xFF if there are remaining bytes
        data += b'\xFF' * remainder
    return data

def bandwidth(uri, packet_size=4, count=500):
    link = cflib.crtp.get_link_driver(uri)

    try:
        # enqueue packets
        start_time = time.time()
        for i in range(count):
            pk = CRTPPacket()
            pk.set_header(CRTPPort.LINKCTRL, 0)  # Echo channel
            pk.data = build_data(i, packet_size)
            if not link.send_packet(pk):
                raise Exception("send_packet() timeout!")

        # get the result
        for i in range(count):
            while True:
                pk_ack = link.receive_packet(2)
                if pk_ack is None:
                    raise Exception("Receive packet timeout!")
                if pk_ack.port == CRTPPort.LINKCTRL and pk_ack.channel == 0:
                    break
            # make sure we actually received the expected value
            i_recv, = struct.unpack('<I', pk_ack.data[0:4])
            assert(i_recv == i)
        end_time = time.time()
    except Exception as e:
        link.close()
        raise e

    link.close()
    result = count / (end_time - start_time)
    logger.info('bandwidth: {}'.format(result))

    return result


def ping(uri, packet_size=4, count=500):
    link = cflib.crtp.get_link_driver(uri)

    try:
        for i in range(count):
            pk = CRTPPacket()
            pk.set_header(CRTPPort.LINKCTRL, 0)  # Echo channel
            pk.data = build_data(i, packet_size)
            if not link.send_packet(pk):
                raise Exception("send_packet() timeout!")
            time.sleep(0.1)
            while True:
                pk_ack = link.receive_packet(2)
                if pk_ack is None:
                    raise Exception("Receive packet timeout!")
                if pk_ack.port == CRTPPort.LINKCTRL and pk_ack.channel == 0:
                    break
            # make sure we actually received the expected value
            assert(pk.data == pk_ack.data)

    finally:
        link.close()

    return True
