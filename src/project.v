`default_nettype none

module tt_um_iir_filter (
    input  wire [7:0] ui_in,    // Data in: x[n]
    output wire [7:0] uo_out,   // Data out: y[n]
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);

    // Ground unused bidirectional pins
    assign uio_out = 8'b0;
    assign uio_oe  = 8'b0;

    // Internal state registers for the delay lines (Z^-1)
    reg signed [7:0] x_1, x_2;
    reg signed [7:0] y_1, y_2;

    // Hardcoded coefficients (Example low-pass configuration)
    wire signed [7:0] b0 = 8'sd12;
    wire signed [7:0] b1 = 8'sd24;
    wire signed [7:0] b2 = 8'sd12;
    wire signed [7:0] a1 = -8'sd10;
    wire signed [7:0] a2 = 8'sd5;

    // Combinational Multiply-Accumulate (MAC) for the Biquad equation
    wire signed [7:0] x_in = ui_in;
    wire signed [15:0] acc = (x_in * b0) + (x_1 * b1) + (x_2 * b2) - (y_1 * a1) - (y_2 * a2);
    
    // Scale the 16-bit accumulator back down to 8-bit output
    wire signed [7:0] y_out = acc[11:4]; 
    assign uo_out = y_out;

    // Sequential logic: Shift the delay lines on every clock tick
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x_1 <= 8'd0; x_2 <= 8'd0;
            y_1 <= 8'd0; y_2 <= 8'd0;
        end else if (ena) begin
            x_1 <= x_in;
            x_2 <= x_1;
            y_1 <= y_out;
            y_2 <= y_1;
        end
    end

endmodule