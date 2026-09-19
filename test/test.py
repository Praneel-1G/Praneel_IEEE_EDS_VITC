# SPDX-FileCopyrightText: © 2026 Tiny Protocol Nexus
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

SYNC = 0xA5
OP_AXI_WR = 0x01
OP_AXI_RD = 0x02
OP_APB_WR = 0x03
OP_APB_RD = 0x04
OP_STATUS = 0x10
OP_ERROR = 0x11
OP_TRACE = 0x12
OP_FAULT = 0x20
OP_CLEAR = 0x21

ST_OK = 0x00
ST_SLVERR = 0x01
ST_TIMEOUT = 0x02
ST_PROTOCOL = 0x03


def u8(x):
    return x & 0xFF


async def pulse_cmd_stb(dut, byte):
    dut.ui_in.value = byte
    dut.uio_in.value = dut.uio_in.value.integer & ~(1 << 4)
    await ClockCycles(dut.clk, 1)
    dut.uio_in.value = dut.uio_in.value.integer | (1 << 4)
    await ClockCycles(dut.clk, 1)
    dut.uio_in.value = dut.uio_in.value.integer & ~(1 << 4)
    await ClockCycles(dut.clk, 1)


async def send_cmd(dut, op, addr=0, data=0, arg=0xF):
    frame = [
        SYNC,
        op,
        addr & 0xFF,
        (addr >> 8) & 0xFF,
        data & 0xFF,
        (data >> 8) & 0xFF,
        (data >> 16) & 0xFF,
        (data >> 24) & 0xFF,
        arg & 0xFF,
    ]
    for b in frame:
        await pulse_cmd_stb(dut, b)


async def read_response(dut, timeout_cycles=100):
    for _ in range(timeout_cycles):
        if dut.uio_out.value.to_unsigned() & 0x01:
            result = []
            for _ in range(5):
                result.append(dut.uo_out.value.to_unsigned())
                dut.uio_in.value = dut.uio_in.value.integer | (1 << 5)
                await ClockCycles(dut.clk, 1)
                dut.uio_in.value = dut.uio_in.value.integer & ~(1 << 5)
                await ClockCycles(dut.clk, 1)
                if _ != 4:
                    await ClockCycles(dut.clk, 1)
            status = result[0]
            data = result[1] | (result[2] << 8) | (result[3] << 16) | (result[4] << 24)
            return status, data
        await ClockCycles(dut.clk, 1)
    raise AssertionError("Timed out waiting for response")


@cocotb.test()
async def test_protocol_nexus(dut):
    dut._log.info("Starting Tiny Protocol Nexus test")

    clock = Clock(dut.clk, 20, unit="ns")
    cocotb.start_soon(clock.start())

    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)

    # Clear all counters/errors.
    await send_cmd(dut, OP_CLEAR)
    status, data = await read_response(dut)
    assert status == ST_OK

    # AXI4-Lite write/read round trip.
    await send_cmd(dut, OP_AXI_WR, addr=0x0010, data=0x12345678, arg=0xF)
    status, data = await read_response(dut)
    assert status == ST_OK, f"AXI write failed: 0x{status:02x}"

    await send_cmd(dut, OP_AXI_RD, addr=0x0010)
    status, data = await read_response(dut)
    assert status == ST_OK
    assert data == 0x12345678, f"AXI readback mismatch: 0x{data:08x}"

    # Byte write strobe through AXI.
    await send_cmd(dut, OP_AXI_WR, addr=0x0010, data=0x0000AA00, arg=0x2)
    status, _ = await read_response(dut)
    assert status == ST_OK
    await send_cmd(dut, OP_AXI_RD, addr=0x0010)
    status, data = await read_response(dut)
    assert status == ST_OK
    assert data == 0x1234AA78, f"WSTRB mismatch: 0x{data:08x}"

    # APB write/read round trip.
    await send_cmd(dut, OP_APB_WR, addr=0x0020, data=0xCAFEBABE, arg=0xF)
    status, _ = await read_response(dut)
    assert status == ST_OK

    await send_cmd(dut, OP_APB_RD, addr=0x0020)
    status, data = await read_response(dut)
    assert status == ST_OK
    assert data == 0xCAFEBABE, f"APB readback mismatch: 0x{data:08x}"

    # Force a slave error.
    fault_cfg = 0x01
    await send_cmd(dut, OP_FAULT, data=fault_cfg)
    status, _ = await read_response(dut)
    assert status == ST_OK

    await send_cmd(dut, OP_AXI_WR, addr=0x0024, data=0x55AA55AA, arg=0xF)
    status, _ = await read_response(dut)
    assert status == ST_SLVERR, f"Expected SLVERR, got 0x{status:02x}"

    # Clear fault injection.
    await send_cmd(dut, OP_FAULT, data=0x00000000)
    status, _ = await read_response(dut)
    assert status == ST_OK

    # Timeout injection: master should terminate instead of hanging forever.
    await send_cmd(dut, OP_FAULT, data=0x00000002)  # bit1 set
    status, _ = await read_response(dut)
    assert status == ST_OK

    await send_cmd(dut, OP_APB_RD, addr=0x0020)
    status, _ = await read_response(dut, timeout_cycles=100)
    assert status == ST_TIMEOUT, f"Expected timeout, got 0x{status:02x}"

    await send_cmd(dut, OP_FAULT, data=0x00000000)
    status, _ = await read_response(dut)
    assert status == ST_OK

    # Status counters must show completed transactions and at least one error.
    await send_cmd(dut, OP_STATUS)
    status, data = await read_response(dut)
    assert status == ST_OK
    total = (data >> 24) & 0xFF
    errors = (data >> 16) & 0xFF
    timeouts = (data >> 8) & 0xFF
    assert total >= 7, f"Unexpected total transaction count: {total}"
    assert errors >= 2, f"Expected errors from SLVERR + timeout, got {errors}"
    assert timeouts >= 1, f"Expected timeout count >=1, got {timeouts}"

    # Error flags should be non-zero.
    await send_cmd(dut, OP_ERROR)
    status, flags = await read_response(dut)
    assert status == ST_OK
    assert flags != 0, "Error flags were unexpectedly clear"

    # Trace: read low and high halves of entry zero; just verify the interface responds.
    await send_cmd(dut, OP_TRACE, addr=0x0000, arg=0x00)
    status, low = await read_response(dut)
    assert status == ST_OK
    await send_cmd(dut, OP_TRACE, addr=0x0000, arg=0x01)
    status, high = await read_response(dut)
    assert status == ST_OK
    assert (low != 0) or (high != 0), "Trace entry was unexpectedly empty"

    dut._log.info("Protocol Nexus test completed successfully")
