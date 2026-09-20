# SPDX-FileCopyrightText: © 2026 Your Name
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

# Python model of the approximate FIR Engine
class ApproxFIR:
    def __init__(self, coeffs):
        self.coeffs = coeffs
        self.delay = [0, 0, 0]

    def approx_mult(self, a, b):
        a_l = a & 0xF
        b_l = b & 0xF
        # Approximation Formula: AB - A_L * B_L
        return (a * b) - (a_l * b_l)

    def process(self, x):
        taps = [x] + self.delay
        acc = 0
        for t, c in zip(taps, self.coeffs):
            acc += self.approx_mult(t, c)
            
        self.delay = [x, self.delay[0], self.delay[1]]
        
        # Scale to match hardware: (Accumulator) >> 8
        return (acc >> 8) & 0xFF


async def spi_transfer(dut, byte_out):
    """Performs an 8-bit SPI Mode 0 transfer with tight timing margins."""
    byte_in = 0
    for i in range(8):
        bit = 1 if (byte_out & (1 << (7 - i))) else 0
        
        # SCLK = 0 (Setup MOSI)
        dut.ui_in.value = (bit << 1) | 0x00
        await ClockCycles(dut.clk, 10)
        
        # SCLK = 1 (RTL samples MOSI on this rising edge)
        dut.ui_in.value = (bit << 1) | 0x01
        
        # --- FIX 12/13: Sample MISO immediately after the rising edge ---
        await ClockCycles(dut.clk, 1) 
        miso_bit = dut.uo_out.value.to_unsigned() & 1
        byte_in = (byte_in << 1) | miso_bit
        
        # Wait a few cycles before falling edge
        await ClockCycles(dut.clk, 4)
        
        # SCLK = 0 (RTL shifts next MISO bit on this falling edge)
        dut.ui_in.value = (bit << 1) | 0x00
        await ClockCycles(dut.clk, 5)
        
    return byte_in


@cocotb.test()
async def test_approximate_mac(dut):
    dut._log.info("Starting Time-Multiplexed Approximate MAC test")

    clock = Clock(dut.clk, 20, unit="ns") # 50 MHz clock
    cocotb.start_soon(clock.start())

    # Initial Reset
    dut.ena.value = 1
    dut.ui_in.value = 0x04 # CS_N = 1 (idle)
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 10)

    # 1. Load Coefficients via SPI
    dut.ui_in.value = 0x00 # Drop CS_N
    await ClockCycles(dut.clk, 10)
    
    await spi_transfer(dut, 0x01) # Command: LOAD_COEFF
    await spi_transfer(dut, 0x0A) # C0 = 10 
    await spi_transfer(dut, 0x0A) # C1 = 10
    await spi_transfer(dut, 0x0A) # C2 = 10
    await spi_transfer(dut, 0x0A) # C3 = 10
    
    dut.ui_in.value = 0x04 # Raise CS_N
    await ClockCycles(dut.clk, 20)

    # --- FIX 14: Dynamic mathematical corner cases ---
    # Testing zeros, maximal values, standard cases, and mixed values
    test_vectors = [
        100, 100, 100, 100,  # Standard: 100 * 10
        0, 0, 0, 0,          # Zero flush
        255, 255, 255, 255,  # Max range accumulator test
        15, 16, 15, 31,      # Approximation boundary cases
        170, 13, 0           # Mixed random
    ]

    model = ApproxFIR([10, 10, 10, 10])

    # 2. Stream Data
    dut.ui_in.value = 0x00 # Drop CS_N
    await ClockCycles(dut.clk, 10)
    await spi_transfer(dut, 0x02) # Command: STREAM_DATA
    
    prev_expected = 0 # First returned byte is always 0 (junk payload)
    
    for i, x in enumerate(test_vectors):
        y_expected = model.process(x)
        
        # SPI is full duplex. The output we receive during 'x' belongs to 'x-1'
        y_actual = await spi_transfer(dut, x)
        
        dut._log.info(f"Vector {i}: Input=0x{x:02X} | Received=0x{y_actual:02X} | Expected=0x{prev_expected:02X}")
        assert y_actual == prev_expected, f"Mismatch at vector {i}: Expected 0x{prev_expected:02X}, got 0x{y_actual:02X}"
        
        prev_expected = y_expected

    dut._log.info("All Approximate MAC corner cases passed successfully!")
    dut.ui_in.value = 0x04 # Raise CS_N
    await ClockCycles(dut.clk, 20)
