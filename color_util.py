def eight_bit_rgb_to_float(c):    
    return (c[0] / 255., c[1] / 255., c[2] / 255.)

def ebr2f(c):
    return eight_bit_rgb_to_float(c)
