"""HMAC (Keyed-Hashing for Message Authentication) Python module.

Implements the HMAC algorithm as described by RFC 2104.
"""
import md5

trans_5C = bytes((x ^ 0x5C) for x in range(256))
trans_36 = bytes((x ^ 0x36) for x in range(256))

def translate(d, t):
    return bytes(t[x] for x in d)

class HMAC:
    def __init__(self, key, msg = None):
        """Create a new HMAC object.

        key:       key for the keyed hash object.
        msg:       Initial input for the hash, if provided.

        Note: key and msg must be a bytes or bytearray objects.
        """

        self.digest_size = 16
        self.block_size = 64

        key = key + bytes(self.block_size - len(key))

        self.outer = translate(key, trans_5C)
        self.inner = translate(key, trans_36)

        self.out = md5.digest(self.inner + msg)

        self.out = md5.digest(self.outer + self.out)
