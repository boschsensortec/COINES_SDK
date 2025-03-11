#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Copyright (c) 2025 Bosch Sensortec GmbH. All rights reserved.
BSD-3-Clause

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
""" 

# This module contains helper functions for binary and sensor data conversions

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

def combine_bytes_to_value(buffer, start_index, bytes_count):
    """
    Combine a specified number of bytes from a buffer into a numeric value.

    Args:
        buffer (list or bytes): The buffer containing the bytes.
        start_index (int): The starting index in the buffer to extract the value.
        bytes_count (int): The number of bytes to combine into the value.

    Returns:
        tuple: A tuple containing the combined numeric value and the updated index.
    """
    if len(buffer) < start_index + bytes_count:
        raise ValueError("Buffer does not have enough bytes to combine into a value.")

    # Use slicing and int.from_bytes for efficient conversion
    value = int.from_bytes(buffer[start_index:start_index + bytes_count], byteorder='big')
    return value, start_index + bytes_count
