`default_nettype none

module tt_um_fir_filter (
    input  wire [7:0] ui_in,    // Data in: x[n]
    output wire [7:0] uo_out,   // Data out: y[n]
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);
    // 1. Ground unused bidirectional pins
    assign uio_out = 8'b0;
    assign uio_oe  = 8'b0;

    // 2. The Delay Line (Shift Register)
    reg signed [7:0] tap0, tap1, tap2, tap3;

    // 3. Hardcoded Filter Coefficients (Weights)
    wire signed [7:0] h0 = 8'sd2;
    wire signed [7:0] h1 = 8'sd4;
    wire signed [7:0] h2 = 8'sd4;
    wire signed [7:0] h3 = 8'sd2;

    // 4. Combinational Multiply and Accumulate
    wire signed [15:0] acc = (tap0 * h0) + (tap1 * h1) + (tap2 * h2) + (tap3 * h3);

    // 5. Scale output to fit 8 bits (Divide by 4 via bit-shifting)
    assign uo_out = acc[9:2];

    // 6. Sequential Logic: Shift the data down the line on every clock tick
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tap0 <= 8'd0;
            tap1 <= 8'd0;
            tap2 <= 8'd0;
            tap3 <= 8'd0;
        end else if (ena) begin
            tap0 <= ui_in;    // Newest data enters the line
            tap1 <= tap0;     // Shift to stage 1
            tap2 <= tap1;     // Shift to stage 2
            tap3 <= tap2;     // Shift to stage 3
        end
    end

endmodule