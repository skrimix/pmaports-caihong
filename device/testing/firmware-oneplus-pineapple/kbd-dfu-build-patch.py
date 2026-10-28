#!/usr/bin/env python3
"""Build a KA026 container with the Search+F4 patch (see search_f4.S).

Usage: build.py STOCK_CONTAINER PATCH_BIN OUT [MARKER]
Rewrites the MCU image (type 2), its CRC16 in the type-1 descriptor and the
container's SHA-256 digests. Version strings are left unchanged.

MARKER (up to 12 ASCII characters) becomes the BLE Device Information Model
Number, stock " ": the Model case of the DISS value handler (sub_40FBE4) is
pointed at the unused Serial Number code and slot, which gets the marker.
"""
import binascii, hashlib, struct, sys

FN_ADDR, FN_SIZE, BASE = 0x408EE0, 0x60, 0x403000
STOCK_SHA = '04640fc7b1d20f2ebc08371c4bfd606450316a97a00053c30b3c01519858f9e7'

DIS_CASE_MODEL = 0x40FBFB    # tbb entry of the Model Number case
DIS_SERIAL_CASE = 0x10       # tbb entry of the Serial Number case
DIS_SERIAL_LEN = 0x40FC1A    # movs r5, #10
DIS_SERIAL_SLOT, DIS_SLOT_SIZE = 0x40FC98, 12

src, patch, out = sys.argv[1:4]
marker = sys.argv[4].encode('ascii') if len(sys.argv) > 4 else None
assert marker is None or (0 < len(marker) <= DIS_SLOT_SIZE and marker.isascii()
                          and all(0x20 <= b < 0x7f for b in marker)), 'bad marker'
c = bytearray(open(src, 'rb').read())
p = open(patch, 'rb').read()
assert len(p) <= FN_SIZE
ents = {}
for i in range(1, c[9] + 1):
    typ, off, ln = struct.unpack_from('>HII', c, i * 64)
    ents[typ] = (i, off, ln)

_, off2, ln2 = ents[2]
img = bytearray(c[off2:off2 + ln2])
assert hashlib.sha256(img).hexdigest() == STOCK_SHA, 'not the 1.5.7 image'
o = FN_ADDR - BASE
assert img[o:o + 2] == b'\xb0\xb5'  # stock push {r4,r5,r7,lr}
img[o:o + FN_SIZE] = p + b'\x00\xbf' * ((FN_SIZE - len(p)) // 2)
if marker:
    t, n, s = DIS_CASE_MODEL - BASE, DIS_SERIAL_LEN - BASE, DIS_SERIAL_SLOT - BASE
    assert img[t] == 0x06 and img[n:n + 2] == b'\x0a\x25'
    assert img[s:s + DIS_SLOT_SIZE] == b'1.0.0.0-LE\0\0'
    img[t] = DIS_SERIAL_CASE
    img[n] = len(marker)
    img[s:s + DIS_SLOT_SIZE] = marker.ljust(DIS_SLOT_SIZE, b'\0')
c[off2:off2 + ln2] = img

_, off1, ln1 = ents[1]
desc = bytearray(c[off1:off1 + ln1])
assert struct.unpack_from('<I', desc, 0x14)[0] == ln2
struct.pack_into('<H', desc, 0x0e, binascii.crc_hqx(bytes(img), 0))
c[off1:off1 + ln1] = desc

for idx, off, ln in ents.values():
    c[idx * 64 + 10:idx * 64 + 42] = hashlib.sha256(c[off:off + ln]).digest()
c[11:43] = hashlib.sha256(c[64:256]).digest()
open(out, 'wb').write(c)
print('crc16 %04x  image sha256 %s' % (binascii.crc_hqx(bytes(img), 0), hashlib.sha256(img).hexdigest()))
