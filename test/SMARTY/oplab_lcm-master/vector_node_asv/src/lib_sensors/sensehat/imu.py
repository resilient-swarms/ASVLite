"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class imu(object):
    __slots__ = ["timestamp", "orientation", "magnetic_compass", "gyroscope", "acceleration", "orientation_string", "magnetic_compass_string", "gyroscope_string", "acceleration_string"]

    def __init__(self):
        self.timestamp = 0.0
        self.orientation = [ 0.0 for dim0 in range(3) ]
        self.magnetic_compass = [ 0.0 for dim0 in range(3) ]
        self.gyroscope = [ 0.0 for dim0 in range(3) ]
        self.acceleration = [ 0.0 for dim0 in range(3) ]
        self.orientation_string = ""
        self.magnetic_compass_string = ""
        self.gyroscope_string = ""
        self.acceleration_string = ""

    def encode(self):
        buf = BytesIO()
        buf.write(imu._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">d", self.timestamp))
        buf.write(struct.pack('>3d', *self.orientation[:3]))
        buf.write(struct.pack('>3d', *self.magnetic_compass[:3]))
        buf.write(struct.pack('>3d', *self.gyroscope[:3]))
        buf.write(struct.pack('>3d', *self.acceleration[:3]))
        __orientation_string_encoded = self.orientation_string.encode('utf-8')
        buf.write(struct.pack('>I', len(__orientation_string_encoded)+1))
        buf.write(__orientation_string_encoded)
        buf.write(b"\0")
        __magnetic_compass_string_encoded = self.magnetic_compass_string.encode('utf-8')
        buf.write(struct.pack('>I', len(__magnetic_compass_string_encoded)+1))
        buf.write(__magnetic_compass_string_encoded)
        buf.write(b"\0")
        __gyroscope_string_encoded = self.gyroscope_string.encode('utf-8')
        buf.write(struct.pack('>I', len(__gyroscope_string_encoded)+1))
        buf.write(__gyroscope_string_encoded)
        buf.write(b"\0")
        __acceleration_string_encoded = self.acceleration_string.encode('utf-8')
        buf.write(struct.pack('>I', len(__acceleration_string_encoded)+1))
        buf.write(__acceleration_string_encoded)
        buf.write(b"\0")

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != imu._get_packed_fingerprint():
            raise ValueError("Decode error")
        return imu._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = imu()
        self.timestamp = struct.unpack(">d", buf.read(8))[0]
        self.orientation = struct.unpack('>3d', buf.read(24))
        self.magnetic_compass = struct.unpack('>3d', buf.read(24))
        self.gyroscope = struct.unpack('>3d', buf.read(24))
        self.acceleration = struct.unpack('>3d', buf.read(24))
        __orientation_string_len = struct.unpack('>I', buf.read(4))[0]
        self.orientation_string = buf.read(__orientation_string_len)[:-1].decode('utf-8', 'replace')
        __magnetic_compass_string_len = struct.unpack('>I', buf.read(4))[0]
        self.magnetic_compass_string = buf.read(__magnetic_compass_string_len)[:-1].decode('utf-8', 'replace')
        __gyroscope_string_len = struct.unpack('>I', buf.read(4))[0]
        self.gyroscope_string = buf.read(__gyroscope_string_len)[:-1].decode('utf-8', 'replace')
        __acceleration_string_len = struct.unpack('>I', buf.read(4))[0]
        self.acceleration_string = buf.read(__acceleration_string_len)[:-1].decode('utf-8', 'replace')
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if imu in parents: return 0
        tmphash = (0x274976aeaf3d6912) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if imu._packed_fingerprint is None:
            imu._packed_fingerprint = struct.pack(">Q", imu._get_hash_recursive([]))
        return imu._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)
