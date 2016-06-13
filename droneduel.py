import array
import struct

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
  #def __init__(self, phase, cid, vid, aileron=1500, elevator=1500, throttle=1500, rudder=1500, flip=0, mode=2, address=[0xCC]*5, crc=0):
  def __init__(self, phase, cid, vid, aileron=1500, elevator=1500, throttle=1500, rudder=1500, flip=0, mode=2, crc=0):
  #def __init__(self, address, phase, cid, vid, aileron=1500, elevator=1500, throttle=1500, rudder=1500, flip=0, mode=2, crc=0):
    self.address = [0xCC, 0xCC, 0xCC, 0xCC, 0xCC]
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


  def calc_channels(self):
    freq = [-1]*4
  
    freq[0] = (self.cid[1] & 0x0F) + 0x03;
    freq[1] = (self.cid[1] >> 4) + 0x16;
    freq[2] = (self.cid[0] & 0x0F) + 0x2D;
    freq[3] = (self.cid[0] >> 4) + 0x40;
  
    return freq


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
    return crc_match, drone_packet(phase, cid, vid, aileron, elevator, throttle, rudder, flip, mode, crc_given)


