# SPDX-FileCopyrightText: © 2026 Your Name
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

async def spi_transfer(dut, byte_out):
    """Performs an 8-bit SPI Mode 0 transfer."""
    byte_in = 0
    for i in range(8):
        # Set MOSI and SCLK = 0
        bit = 1 if (byte_out & (1 << (7 - i))) else 0
        dut.ui_in.value = (bit << 1) | 0x00  # CS_N=0, MOSI=bit, SCLK=0
        await ClockCycles(dut.clk, 10)
        
        # SCLK = 1 (RTL samples on rising edge)
        dut.ui_in.value = (bit << 1) | 0x01
        await ClockCycles(dut.clk, 10)
        
        # Capture MISO using modern to_unsigned() 
        miso_bit = dut.uo_out.value.to_unsigned() & 1
        byte_in = (byte_in << 1) | miso_bit
        
        # SCLK = 0 (RTL prepares next bit on falling edge)
        dut.ui_in.value = (bit << 1) | 0x00
        await ClockCycles(dut.clk, 5)
        
    return byte_in

@cocotb.test()
async def test_approximate_mac(dut):
    dut._log.info("Starting Time-Multiplexed Approximate MAC test")

    # Fixed syntax: 'unit' instead of 'units'
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

    # ----------------------------------------------------
    # 1. Load Coefficients via SPI
    # ----------------------------------------------------
    dut.ui_in.value = 0x00 # Drop CS_N
    await ClockCycles(dut.clk, 10)
    
    await spi_transfer(dut, 0x01) # Command: LOAD_COEFF
    await spi_transfer(dut, 0x0A) # C0 = 10 (0x0A)
    await spi_transfer(dut, 0x0A) # C1 = 10
    await spi_transfer(dut, 0x0A) # C2 = 10
    await spi_transfer(dut, 0x0A) # C3 = 10
    
    dut.ui_in.value = 0x04 # Raise CS_N
    await ClockCycles(dut.clk, 20)

    # ----------------------------------------------------
    # 2. Stream Data and Verify Approximation Mathematics
    # ----------------------------------------------------
    dut.ui_in.value = 0x00 # Drop CS_N
    await ClockCycles(dut.clk, 10)
    
    await spi_transfer(dut, 0x02) # Command: STREAM_DATA
    
    # Send a constant input of 100 (0x64)
    # 
    # Exact Math: 100 * 10 = 1000 (0x03E8)
    # Approximate Math (Missing 4x4 LSB):
    # A=0x64 (A_high=6, A_low=4), B=0x0A (B_high=0, B_low=10)
    # Approx Prod = (6*0)<<8 + (6*10)<<4 + (4*0)<<4 = 960 (0x03C0)
    
    # Tap 1 (Time step 0) -> Sum = 960. MISO should be 960 >> 8 = 3 (0x03)
    res1 = await spi_transfer(dut, 0x64) 
    
    # Tap 2 -> Sum = 960 * 2 = 1920 (0x0780). MISO = 0x07
    res2 = await spi_transfer(dut, 0x64) 
    assert res2 == 0x03, f"Expected 0x03, got 0x{res2:02X}"
    
    # Tap 3 -> Sum = 960 * 3 = 2880 (0x0B40). MISO = 0x0B
    res3 = await spi_transfer(dut, 0x64)
    assert res3 == 0x07, f"Expected 0x07, got 0x{res3:02X}"
    
    # Tap 4 -> Sum = 960 * 4 = 3840 (0x0F00). MISO = 0x0F
    res4 = await spi_transfer(dut, 0x64)
    assert res4 == 0x0B, f"Expected 0x0B, got 0x{res4:02X}"

    # Tap 5 (Steady State Full Pipeline) -> MISO = 0x0F
    res5 = await spi_transfer(dut, 0x64)
    assert res5 == 0x0F, f"Expected 0x0F, got 0x{res5:02X}"
    
    dut._log.info("Approximate MAC Math verified successfully!")
    dut.ui_in.value = 0x04 # Raise CS_N
    await ClockCycles(dut.clk, 20)
