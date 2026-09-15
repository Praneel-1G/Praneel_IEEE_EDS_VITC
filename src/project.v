`default_nettype none

module tt_um_counter (
    input  wire [7:0] ui_in,    // Unused input
    output wire [7:0] uo_out,   // Counter output
    input  wire [7:0] uio_in,   // Unused bidirectional input
    output wire [7:0] uio_out,  // Grounded
    output wire [7:0] uio_oe,   // Grounded
    input  wire       ena,      // Counter enable
    input  wire       clk,      // Clock signal
    input  wire       rst_n     // Active-low asynchronous reset
);

    // 1. Ground unused bidirectional pins
    assign uio_out = 8'b0;
    assign uio_oe  = 8'b0;

    // 2. Counter Register
    reg [7:0] count_reg;

    // 3. Assign register value to output pins
    assign uo_out = count_reg;

    // 4. Sequential Counter Logic
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            count_reg <= 8'd0;
        end else if (ena) begin
            count_reg <= count_reg + 1'b1;
        end
    end

endmodule
