import pytest
import time
import struct

import numpy as np

import cflib.crtp
from cflib.crtp.crtpstack import CRTPPacket
from cflib.crtp.crtpstack import CRTPPort

import conftest

@pytest.mark.parametrize('dev', conftest.get_devices(), ids=lambda d: d.name)
class TestRadio:
    def test_latency_small_packets(self, dev):
        requirement = conftest.get_requirement('radio', 'latencysmall')
        assert(latency(dev.link_uri, requirement['packet_size']) < requirement['limit_high_ms'])


    def test_latency_big_packets(self, dev):
        requirement = conftest.get_requirement('radio', 'latencybig')
        assert(latency(dev.link_uri, requirement['packet_size']) < requirement['limit_high_ms'])

    def test_bandwidth_small_packets(self, dev):
        requirement = conftest.get_requirement('radio', 'bwsmall')
        assert(bandwidth(dev.link_uri, requirement['packet_size']) > requirement['limit_low'])

    def test_bandwidth_big_packets(self, dev):
        requirement = conftest.get_requirement('radio', 'bwbig')
        assert(bandwidth(dev.link_uri, requirement['packet_size']) > requirement['limit_low'])
    
    def test_reliability(self, dev):
        requirement = conftest.get_requirement('radio', 'reliability')
        # The bandwidth function will fail if there is any packet loss
        bandwidth(dev.link_uri, 4, requirement['limit_low'])


def build_data(i, packet_size):
    assert(packet_size % 4 == 0)
    repeats = packet_size // 4
    return struct.pack('<' + 'I'*repeats, *[i]*repeats)


def latency(uri, packet_size=4, count=500):
    link = cflib.crtp.get_link_driver(uri)

    pk = CRTPPacket()
    pk.set_header(CRTPPort.LINKCTRL, 0)  # Echo channel

    latencies = []
    for i in range(count):
        pk.data = build_data(i, packet_size)

        start_time = time.time()
        link.send_packet(pk)
        while True:
            pk_ack = link.receive_packet(-1)
            if pk_ack.port == CRTPPort.LINKCTRL and pk_ack.channel == 0:
                break
        end_time = time.time()

        # make sure we actually received the expected value
        i_recv, = struct.unpack('<I', pk_ack.data[0:4])
        assert(i == i_recv)
        latencies.append((end_time - start_time) * 1000)
    link.close()
    result = np.min(latencies)
    return result


def bandwidth(uri, packet_size=4, count=500):
    link = cflib.crtp.get_link_driver(uri)

    # enqueue packets
    start_time = time.time()
    for i in range(count):
        pk = CRTPPacket()
        pk.set_header(CRTPPort.LINKCTRL, 0)  # Echo channel
        pk.data = build_data(i, packet_size)
        link.send_packet(pk)

    # get the result
    for i in range(count):
        while True:
            pk_ack = link.receive_packet(-1)
            if pk_ack.port == CRTPPort.LINKCTRL and pk_ack.channel == 0:
                break
        # make sure we actually received the expected value
        i_recv, = struct.unpack('<I', pk_ack.data[0:4])
        assert(i_recv == i)
    end_time = time.time()
    link.close()
    result = count / (end_time - start_time)
    return result
