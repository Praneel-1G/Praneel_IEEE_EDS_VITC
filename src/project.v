/*
 * Approximate DSP: Time-Multiplexed MAC Coprocessor
 * SPI-driven FIR engine with structural multiplication approximation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tt_um_approx_mac_coprocessor (
    input  wire [7:0] ui_in,    // Dedicated inputs (SPI)
    output wire [7:0] uo_out,   // Dedicated outputs (MISO & Debug)
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high)
    input  wire       ena,      // always 1 when the design is powered
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

    // --- FIX 10: Safely tie off unused signals ---
    wire _unused = &{
        ena,
        uio_in,
        ui_in[7:3],
        1'b0
    };

    // --- FIX 2: Correct IO Enable values for "unused" status ---
    assign uio_oe  = 8'h00; 
    assign uio_out = 8'h00;

    // Pin mappings
    wire sclk = ui_in[0];
    wire mosi = ui_in[1];
    wire cs_n = ui_in[2];

    // SPI edge detection via system clock oversampling
    reg [2:0] sclk_r;
    reg [1:0] mosi_r;
    reg [1:0] cs_n_r; 

    always @(posedge clk) begin
        if (!rst_n) begin
            sclk_r <= 3'b0;
            mosi_r <= 2'b0;
            cs_n_r <= 2'b11;
        end else begin
            sclk_r <= {sclk_r[1:0], sclk};
            mosi_r <= {mosi_r[0], mosi};
            cs_n_r <= {cs_n_r[0], cs_n};
        end
    end

    wire sclk_rise = (sclk_r[2:1] == 2'b01);
    wire sclk_fall = (sclk_r[2:1] == 2'b10);
    wire cs_active = ~cs_n_r[1];

    // SPI Receiver State
    reg [2:0] bit_cnt;
    reg [7:0] rx_shift;
    reg [7:0] rx_byte;
    reg       rx_ready;

    always @(posedge clk) begin
        if (!rst_n || !cs_active) begin
            bit_cnt  <= 0;
            rx_ready <= 0;
            rx_shift <= 0;
        end else begin
            rx_ready <= 0;
            if (sclk_rise) begin
                rx_shift <= {rx_shift[6:0], mosi_r[1]};
                if (bit_cnt == 7) begin
                    rx_byte  <= {rx_shift[6:0], mosi_r[1]};
                    rx_ready <= 1;
                    bit_cnt  <= 0;
                end else begin
                    bit_cnt  <= bit_cnt + 1;
                end
            end
        end
    end

    // Coprocessor FSM
    localparam IDLE       = 2'd0;
    localparam LOAD_COEFF = 2'd1;
    localparam STREAM     = 2'd2;

    reg [1:0] state;
    reg [1:0] coeff_cnt;
    reg [7:0] C [0:3];   // Coefficient Memory
    reg [7:0] D [0:2];   // Data Delay Line

    always @(posedge clk) begin
        if (!rst_n || !cs_active) begin
            state     <= IDLE;
            coeff_cnt <= 0;
        end else if (rx_ready) begin
            case (state)
                IDLE: begin
                    if (rx_byte == 8'h01) state <= LOAD_COEFF;
                    else if (rx_byte == 8'h02) state <= STREAM;
                end
                LOAD_COEFF: begin
                    C[coeff_cnt] <= rx_byte;
                    coeff_cnt    <= coeff_cnt + 1;
                    if (coeff_cnt == 3) state <= IDLE;
                end
                STREAM: begin
                    // Remain in STREAM until CS goes high
                end
            endcase
        end
    end

    // Time-Multiplexed Approximate MAC Engine
    reg [2:0]  mac_state;
    reg [17:0] acc;       // FIX 6/7: Expanded to 18-bit for safety against 255*255*4
    reg [7:0]  tx_data;

    reg [7:0]  mul_a;
    reg [7:0]  mul_b;

    // --- FIX 4/5: EXPLICIT 8-BIT PARTIAL PRODUCTS ---
    // Forces Verilog to retain the width required for intermediate math
    wire [7:0] pp_hh = mul_a[7:4] * mul_b[7:4];
    wire [7:0] pp_hl = mul_a[7:4] * mul_b[3:0];
    wire [7:0] pp_lh = mul_a[3:0] * mul_b[7:4];
    
    wire [15:0] mul_out = {pp_hh, 8'b0} + {4'b0, pp_hl, 4'b0} + {4'b0, pp_lh, 4'b0};
    
    // --- FIX 8: Explicit Accumulator behavior ---
    wire [17:0] acc_next = acc + {{2{1'b0}}, mul_out};

    always @(*) begin
        case (mac_state)
            1: begin mul_a = rx_byte; mul_b = C[0]; end
            2: begin mul_a = D[0];    mul_b = C[1]; end
            3: begin mul_a = D[1];    mul_b = C[2]; end
            4: begin mul_a = D[2];    mul_b = C[3]; end
            default: begin mul_a = 8'b0; mul_b = 8'b0; end
        endcase
    end

    always @(posedge clk) begin
        if (!rst_n || !cs_active) begin
            mac_state <= 0;
            acc       <= 0;
            tx_data   <= 0;
            D[0] <= 0; D[1] <= 0; D[2] <= 0;
        end else begin
            case (mac_state)
                0: if (rx_ready && state == STREAM) mac_state <= 1;
                1: begin acc <= {{2{1'b0}}, mul_out}; mac_state <= 2; end
                2: begin acc <= acc_next;             mac_state <= 3; end
                3: begin acc <= acc_next;             mac_state <= 4; end
                4: begin
                    // Scale to 8 bits using expected divide-by-256 truncation mapping
                    tx_data <= acc_next[15:8]; 
                    
                    // Shift Data Delay Line
                    D[0] <= rx_byte;
                    D[1] <= D[0];
                    D[2] <= D[1];
                    mac_state <= 0;
                end
            endcase
        end
    end

    // SPI Transmitter (MISO)
    reg [7:0] tx_shift;

    always @(posedge clk) begin
        if (!rst_n || !cs_active) begin
            tx_shift <= 0;
        // FIX 3: Removed invalid cs_n_r[2:1] branch
        end else if (mac_state == 4) begin
            tx_shift <= acc_next[15:8];  // Loads computation for next SPI shift cycle
        end else if (sclk_fall) begin
            if (bit_cnt != 0) 
                tx_shift <= {tx_shift[6:0], 1'b0};
        end
    end

    assign uo_out[0]   = tx_shift[7];
    assign uo_out[1]   = (state == LOAD_COEFF);
    assign uo_out[2]   = (state == STREAM);
    assign uo_out[7:3] = 5'b0;

endmodule
