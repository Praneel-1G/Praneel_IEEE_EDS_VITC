# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge

@cocotb.test()
async def test_counter(dut):
    dut._log.info("Starting counter test...")

    # Set the clock period to 20ns (50MHz)
    clock = Clock(dut.clk, 20, unit="ns")
    cocotb.start_soon(clock.start())

    # Reset phase
    dut._log.info("Resetting design...")
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 5) # Hold reset for 5 cycles
    
    # Release reset
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 1)
    
    # Verify counter starts at 0 after reset (.to_unsigned() replaces .integer)
    assert dut.uo_out.value.to_unsigned() == 0, f"Expected 0 after reset, got {dut.uo_out.value.to_unsigned()}"

    dut._log.info("Testing counter increment behavior...")

    # Run for 20 clock cycles and verify the count increments perfectly each time
    for i in range(1, 21):
        await ClockCycles(dut.clk, 1)
        current_count = dut.uo_out.value.to_unsigned()
        dut._log.info(f"Cycle {i}: Counter Output = {current_count}")
        
        # Check that the output matches the expected sequential count
        assert current_count == i, f"Count mismatch! Expected {i}, got {current_count}"
        
    dut._log.info("Testing clock enable (ena = 0) behavior...")
    
    # FIX: Wait for a falling edge before driving control signals to avoid setup/hold race conditions
    await FallingEdge(dut.clk)
    dut.ena.value = 0
    
    # Capture the count value right after disabling it
    last_count = dut.uo_out.value.to_unsigned()
    
    # Wait a few cycles and verify it holds its state completely
    await ClockCycles(dut.clk, 3)
    final_count = dut.uo_out.value.to_unsigned()
    assert final_count == last_count, f"Counter incremented while ena was low! Held {final_count} instead of {last_count}"

    dut._log.info("Test complete. Counter works perfectly!")
