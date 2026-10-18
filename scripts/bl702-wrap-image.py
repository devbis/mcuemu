#!/usr/bin/env python3
"""Wrap a raw BL702 XIP binary into a bflb-style flash image (what bflb-mcu-tool --build writes).

Layout: a 176-byte boot header (BFNP, the 'FCFG' flash configuration from a flash .conf with its CRC-32,
the clock configuration), one 16-byte segment header (load address, length, CRCs left zero) and the binary.
Field offsets and bit positions come from bflb-mcu-tool's bl702 bootheader_cfg_keys.py, read with ast (the
file is not executed).
"""
import ast, binascii, configparser, struct, sys

raw_path, out_path, entry, load, conf_path, keys_path = sys.argv[1], sys.argv[2], int(sys.argv[3], 0), int(sys.argv[4], 0), sys.argv[5], sys.argv[6]

tree = ast.parse(open(keys_path).read())
keys = None
for node in tree.body:
    if isinstance(node, ast.Assign) and getattr(node.targets[0], 'id', '') == 'bootheader_cfg_keys':
        keys = ast.literal_eval(node.value)
assert keys is not None

hdr = bytearray(176)
def put(name, value):
    k = keys[name]
    off, pos, bits = int(k['offset']), int(k['pos']), int(k['bitlen'])
    word = struct.unpack_from('<I', hdr, off)[0]
    mask = ((1 << bits) - 1) << pos
    word = (word & ~mask) | ((value << pos) & mask)
    struct.pack_into('<I', hdr, off, word)

conf = configparser.ConfigParser()
conf.read(conf_path)
for name, text in conf['FLASH_CFG'].items():
    if name in keys and not name.startswith('flashcfg'):
        put(name, int(text, 0))
put('magic_code', 0x504e4642)
put('revision', 1)
put('flashcfg_magic_code', 0x47464346)                      # FCFG (bytes F C F G)
put('flashcfg_crc32', binascii.crc32(bytes(hdr[12:96])))
put('clkcfg_magic_code', 0x47464c43)                        # clock configuration magic
put('xtal_type', 1); put('pll_clk', 1); put('hclk_div', 0); put('bclk_div', 0)
put('flash_clk_type', 1); put('flash_clk_div', 0)
put('clkcfg_crc32', binascii.crc32(bytes(hdr[104:112])))
put('img_len', 1)                                            # segment count
put('bootentry', entry)
put('img_start', load)

data = open(raw_path, 'rb').read()
data += b'\0' * (-len(data) % 16)
seg = struct.pack('<IIII', load, len(data), 0, 0)
open(out_path, 'wb').write(bytes(hdr) + seg + data)
print('wrote', out_path, 'bytes', 176 + 16 + len(data), 'entry', hex(entry), 'load', hex(load))
