""" This module contains helper functions for binary and sensor data conversions"""

def twos_comp(val, bits):
    """ Compute the 2's complement of int value val """
    if (val & (1 << (bits - 1))) != 0:  # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)  # compute negative value
    return val  # return positive value as is

def set_bit_val_0(reg_val, mask):
    """ Sets bit value at index 0 """
    return reg_val & ~mask

def set_bits_pos_0(reg_val, mask, field_val):
    """ Sets bits value from bit index 0 """
    return  (reg_val & ~(mask)) | (field_val & mask)

def set_bits(reg_val, mask, field_val, position):
    """ Sets bits value at the given position """
    return  (reg_val & ~(mask)) | ((field_val << position) & mask)
