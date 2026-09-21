`default_nettype none
`timescale 1ns / 1ps

module tb ();

    // Wire up the inputs and outputs
    reg  clk;
    reg  rst_n;
    reg  ena;
    reg  [7:0] ui_in;
    reg  [7:0] uio_in;
    wire [7:0] uo_out;
    wire [7:0] uio_out;
    wire [7:0] uio_oe;

    tt_um_approx_mac_coprocessor user_project (
        // --- POWER PINS COMPLETELY REMOVED ---
        // The IHP gate-level netlist does not expose them on the user boundary.
        .ui_in  (ui_in),
        .uo_out (uo_out),
        .uio_in (uio_in),
        .uio_out(uio_out),
        .uio_oe (uio_oe),
        .ena    (ena),
        .clk    (clk),
        .rst_n  (rst_n)
    );

    initial begin
        $dumpfile("tb.fst");
        $dumpvars(0, tb);
        #1;
    end

endmodule
