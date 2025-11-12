import smbus

bus = smbus.SMBus(1)

def read_linear11(addr, cmd):
    try:
        d = bus.read_i2c_block_data(addr, cmd, 2)
        raw = (d[1] << 8) | d[0]
        exp = ((raw >> 11) & 0x1F)
        if exp > 15: exp -= 32
        mant = raw & 0x7FF
        if mant > 1023: mant -= 2048
        return round(mant * (2 ** exp), 3)
    except:
        return None

def read_word(addr, cmd):
    try:
        d = bus.read_word_data(addr, cmd)
        return f"0x{d:04X}"
    except:
        return None

print(f"{'ADDR':<6} {'VOUT (V)':<10} {'IOUT (A)':<10} {'STATUS_WORD'}")
print("-" * 40)

for addr in range(0x58, 0x70):
    vout = read_linear11(addr, 0x8B)
    iout = read_linear11(addr, 0x8C)
    status = read_word(addr, 0x79)
    if vout is not None or iout is not None:
        print(f"0x{addr:02X}   {str(vout):<10} {str(iout):<10} {status}")

