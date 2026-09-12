`default_nettype none

module tt_um_multiplier (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

    // 1. Ground unused bidirectional pins to prevent OpenLane errors
    assign uio_out = 8'b0;
    assign uio_oe  = 8'b0;

    // 2. Split the 8-bit input bus into two 4-bit operands
    wire [3:0] a = ui_in[3:0];
    wire [3:0] b = ui_in[7:4];

    // 3. Perform the multiplication and assign to the 8-bit output bus
    assign uo_out = a * b;

endmodule