/*
 * Tiny Protocol Nexus
 * AXI4-Lite + APB4 protocol engine, checker, trace buffer and fault injector
 *
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tt_um_protocol_nexus (
    input  wire [7:0] ui_in,      // Host command/data byte
    output wire [7:0] uo_out,     // Response byte
    input  wire [7:0] uio_in,     // uio[4]=cmd_stb, uio[5]=rsp_ready
    output wire [7:0] uio_out,    // uio[0]=rsp_valid, [1]=busy, [2]=error, [3]=trace_valid
    output wire [7:0] uio_oe,     // uio[3:0] outputs, uio[7:4] inputs
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);

    // -------------------------------------------------------------------------
    // Host-side command protocol
    // -------------------------------------------------------------------------
    // Every command is exactly 9 bytes:
    //   0: A5 sync
    //   1: opcode
    //   2: addr[7:0]
    //   3: addr[15:8]
    //   4: data[7:0]
    //   5: data[15:8]
    //   6: data[23:16]
    //   7: data[31:24]
    //   8: wstrb / argument byte
    // Host pulses uio_in[4] for one clock for each byte.
    // Response is five bytes: status, data[7:0], data[15:8], data[23:16], data[31:24].
    // Host pulses uio_in[5] to consume each response byte.

    localparam [7:0] SYNC        = 8'hA5;
    localparam [7:0] OP_AXI_WR   = 8'h01;
    localparam [7:0] OP_AXI_RD   = 8'h02;
    localparam [7:0] OP_APB_WR   = 8'h03;
    localparam [7:0] OP_APB_RD   = 8'h04;
    localparam [7:0] OP_STATUS   = 8'h10;
    localparam [7:0] OP_ERROR    = 8'h11;
    localparam [7:0] OP_TRACE    = 8'h12;
    localparam [7:0] OP_FAULT    = 8'h20;
    localparam [7:0] OP_CLEAR    = 8'h21;

    localparam [7:0] ST_OK       = 8'h00;
    localparam [7:0] ST_SLVERR   = 8'h01;
    localparam [7:0] ST_TIMEOUT  = 8'h02;
    localparam [7:0] ST_PROTOCOL = 8'h03;
    localparam [7:0] ST_BADOP    = 8'hF0;
    localparam [7:0] ST_BUSY     = 8'hF1;

    reg        cmd_stb_d;
    reg        rx_active;
    reg [3:0]  rx_count;
    reg [7:0]  rx_op;
    reg [15:0] rx_addr;
    reg [31:0] rx_data;
    reg [7:0]  rx_arg;

    reg        host_req_valid;
    reg        host_req_apb;
    reg        host_req_write;
    reg [15:0] host_req_addr;
    reg [31:0] host_req_data;
    reg [3:0]  host_req_strb;

    // -------------------------------------------------------------------------
    // Fault configuration
    // data[0][0] = force bus error
    // data[0][1] = suppress completion until engine timeout
    // data[1][3:0] = artificial launch delay (0..15 cycles)
    // -------------------------------------------------------------------------
    reg        fault_force_error;
    reg        fault_timeout;
    reg [3:0]  fault_delay;

    // -------------------------------------------------------------------------
    // Scratchpad register file (32 x 32-bit words)
    // Valid bus addresses: 0x0000 .. 0x007C, word aligned.
    // -------------------------------------------------------------------------
    reg [31:0] scratch [0:31];
    reg        rf_wr_en;
    reg [15:0] rf_wr_addr;
    reg [31:0] rf_wr_data;
    reg [3:0]  rf_wr_strb;
    reg [15:0] rf_rd_addr;
    wire [31:0] rf_rd_data;

    wire rf_rd_valid = (rf_rd_addr[15:7] == 9'd0) && (rf_rd_addr[1:0] == 2'b00);
    assign rf_rd_data = rf_rd_valid ? scratch[rf_rd_addr[6:2]] : 32'h0;

    integer si;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (si = 0; si < 32; si = si + 1)
                scratch[si] <= 32'h0;
        end else if (rf_wr_en && (rf_wr_addr[15:7] == 9'd0) && (rf_wr_addr[1:0] == 2'b00)) begin
            if (rf_wr_strb[0]) scratch[rf_wr_addr[6:2]][7:0]   <= rf_wr_data[7:0];
            if (rf_wr_strb[1]) scratch[rf_wr_addr[6:2]][15:8]  <= rf_wr_data[15:8];
            if (rf_wr_strb[2]) scratch[rf_wr_addr[6:2]][23:16] <= rf_wr_data[23:16];
            if (rf_wr_strb[3]) scratch[rf_wr_addr[6:2]][31:24] <= rf_wr_data[31:24];
        end
    end

    // -------------------------------------------------------------------------
    // Transaction engine
    // -------------------------------------------------------------------------
    wire        eng_busy;
    reg         eng_start;
    reg         eng_apb;
    reg         eng_write;
    reg [15:0]  eng_addr;
    reg [31:0]  eng_wdata;
    reg [3:0]   eng_wstrb;

    wire        eng_done;
    wire [7:0]  eng_status;
    wire [31:0] eng_rdata;
    wire [15:0] eng_trace_addr;
    wire [31:0] eng_trace_data;
    wire [2:0]  eng_trace_meta;
    wire [1:0]  eng_trace_status;
    wire [5:0]  eng_latency;
    wire        eng_protocol_error;
    wire        eng_timeout;

    pnx_bus_engine bus_engine (
        .clk(clk),
        .rst_n(rst_n),
        .start(eng_start),
        .apb_sel(eng_apb),
        .write(eng_write),
        .addr(eng_addr),
        .wdata(eng_wdata),
        .wstrb(eng_wstrb),
        .fault_force_error(fault_force_error),
        .fault_timeout(fault_timeout),
        .fault_delay(fault_delay),
        .busy(eng_busy),
        .done(eng_done),
        .status(eng_status),
        .rdata(eng_rdata),
        .trace_addr(eng_trace_addr),
        .trace_data(eng_trace_data),
        .trace_meta(eng_trace_meta),
        .trace_status(eng_trace_status),
        .latency(eng_latency),
        .protocol_error(eng_protocol_error),
        .timeout_seen(eng_timeout),
        .rf_wr_en(rf_wr_en),
        .rf_wr_addr(rf_wr_addr),
        .rf_wr_data(rf_wr_data),
        .rf_wr_strb(rf_wr_strb),
        .rf_rd_addr(rf_rd_addr),
        .rf_rd_data(rf_rd_data)
    );

    // -------------------------------------------------------------------------
    // Counters / trace memory / error state
    // -------------------------------------------------------------------------
    reg [11:0] cycle_counter;
    reg [7:0]  total_count;
    reg [7:0]  read_count;
    reg [7:0]  write_count;
    reg [7:0]  error_count;
    reg [7:0]  timeout_count;
    reg [7:0]  protocol_count;
    reg [5:0]  max_latency;
    reg [7:0]  error_flags;

    reg [63:0] trace_mem [0:7];
    reg [2:0]  trace_wr_ptr;
    wire       trace_valid = (trace_wr_ptr != 3'd0) || (total_count != 8'd0);

    wire [31:0] status_word = {
        total_count,
        error_count,
        timeout_count,
        5'd0,
        trace_wr_ptr
    };

    wire [31:0] error_word = {
        24'h0,
        error_flags
    };

    // -------------------------------------------------------------------------
    // Response holding register
    // -------------------------------------------------------------------------
    reg        rsp_valid;
    reg [2:0]  rsp_index;
    reg [7:0]  rsp_buf0, rsp_buf1, rsp_buf2, rsp_buf3, rsp_buf4;

    assign uo_out =
        (rsp_index == 3'd0) ? rsp_buf0 :
        (rsp_index == 3'd1) ? rsp_buf1 :
        (rsp_index == 3'd2) ? rsp_buf2 :
        (rsp_index == 3'd3) ? rsp_buf3 : rsp_buf4;

    assign uio_out = {
        4'b0000,
        trace_valid,
        (|error_flags),
        eng_busy,
        rsp_valid
    };
    assign uio_oe = 8'b0000_1111;

    wire cmd_stb = uio_in[4];
    wire rsp_ready = uio_in[5];
    wire cmd_stb_rise = cmd_stb & ~cmd_stb_d;

    // -------------------------------------------------------------------------
    // Main control plane
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cmd_stb_d      <= 1'b0;
            rx_active      <= 1'b0;
            rx_count       <= 4'd0;
            rx_op          <= 8'd0;
            rx_addr        <= 16'd0;
            rx_data        <= 32'd0;
            rx_arg         <= 8'd0;

            host_req_valid <= 1'b0;
            host_req_apb   <= 1'b0;
            host_req_write <= 1'b0;
            host_req_addr  <= 16'd0;
            host_req_data  <= 32'd0;
            host_req_strb  <= 4'hF;
            eng_start      <= 1'b0;

            fault_force_error <= 1'b0;
            fault_timeout     <= 1'b0;
            fault_delay       <= 4'd0;

            cycle_counter  <= 12'd0;
            total_count    <= 8'd0;
            read_count     <= 8'd0;
            write_count    <= 8'd0;
            error_count    <= 8'd0;
            timeout_count  <= 8'd0;
            protocol_count <= 8'd0;
            max_latency    <= 6'd0;
            error_flags    <= 8'd0;
            trace_wr_ptr   <= 3'd0;

            rsp_valid <= 1'b0;
            rsp_index <= 3'd0;
            rsp_buf0  <= 8'd0;
            rsp_buf1  <= 8'd0;
            rsp_buf2  <= 8'd0;
            rsp_buf3  <= 8'd0;
            rsp_buf4  <= 8'd0;
        end else if (ena) begin
            cmd_stb_d <= cmd_stb;
            eng_start <= 1'b0;

            if (cycle_counter == 12'hFFF)
                cycle_counter <= 12'd0;
            else
                cycle_counter <= cycle_counter + 12'd1;

            // Response handshake: response data is held until the host consumes it.
            if (rsp_valid && rsp_ready) begin
                if (rsp_index == 3'd4) begin
                    rsp_valid <= 1'b0;
                    rsp_index <= 3'd0;
                end else begin
                    rsp_index <= rsp_index + 3'd1;
                end
            end

            // Launch an internally pending bus request only when the response port
            // is idle. This keeps the external protocol deterministic.
            if (host_req_valid && !eng_busy && !rsp_valid) begin
                eng_apb   <= host_req_apb;
                eng_write <= host_req_write;
                eng_addr  <= host_req_addr;
                eng_wdata <= host_req_data;
                eng_wstrb <= host_req_strb;
                eng_start <= 1'b1;
                host_req_valid <= 1'b0;
            end

            // Command byte framing.
            if (cmd_stb_rise) begin
                if (!rx_active) begin
                    if (ui_in == SYNC) begin
                        rx_active <= 1'b1;
                        rx_count  <= 4'd0;
                    end else begin
                        error_flags[3] <= 1'b1; // framing error
                    end
                end else begin
                    case (rx_count)
                        4'd0: begin rx_op <= ui_in; rx_count <= 4'd1; end
                        4'd1: begin rx_addr[7:0] <= ui_in; rx_count <= 4'd2; end
                        4'd2: begin rx_addr[15:8] <= ui_in; rx_count <= 4'd3; end
                        4'd3: begin rx_data[7:0] <= ui_in; rx_count <= 4'd4; end
                        4'd4: begin rx_data[15:8] <= ui_in; rx_count <= 4'd5; end
                        4'd5: begin rx_data[23:16] <= ui_in; rx_count <= 4'd6; end
                        4'd6: begin rx_data[31:24] <= ui_in; rx_count <= 4'd7; end
                        4'd7: begin
                            rx_arg <= ui_in;
                            rx_active <= 1'b0;

                            // Nine-byte command has now arrived. Address/data registers
                            // contain all previous bytes; ui_in is the argument byte.
                            case (rx_op)
                                OP_AXI_WR, OP_AXI_RD, OP_APB_WR, OP_APB_RD: begin
                                    if (eng_busy || host_req_valid || rsp_valid) begin
                                        rsp_buf0 <= ST_BUSY;
                                        rsp_buf1 <= 8'd0;
                                        rsp_buf2 <= 8'd0;
                                        rsp_buf3 <= 8'd0;
                                        rsp_buf4 <= 8'd0;
                                        rsp_index <= 3'd0;
                                        rsp_valid <= 1'b1;
                                    end else begin
                                        host_req_valid <= 1'b1;
                                        host_req_apb   <= (rx_op == OP_APB_WR) || (rx_op == OP_APB_RD);
                                        host_req_write <= (rx_op == OP_AXI_WR) || (rx_op == OP_APB_WR);
                                        host_req_addr  <= rx_addr;
                                        host_req_data  <= rx_data;
                                        host_req_strb  <= ui_in[3:0];
                                    end
                                end

                                OP_STATUS: begin
                                    rsp_buf0 <= ST_OK;
                                    rsp_buf1 <= status_word[7:0];
                                    rsp_buf2 <= status_word[15:8];
                                    rsp_buf3 <= status_word[23:16];
                                    rsp_buf4 <= status_word[31:24];
                                    rsp_index <= 3'd0;
                                    rsp_valid <= 1'b1;
                                end

                                OP_ERROR: begin
                                    rsp_buf0 <= ST_OK;
                                    rsp_buf1 <= error_word[7:0];
                                    rsp_buf2 <= error_word[15:8];
                                    rsp_buf3 <= error_word[23:16];
                                    rsp_buf4 <= error_word[31:24];
                                    rsp_index <= 3'd0;
                                    rsp_valid <= 1'b1;
                                end

                                OP_TRACE: begin
                                    rsp_buf0 <= ST_OK;
                                    rsp_buf1 <= (ui_in[0] ? trace_mem[rx_addr[2:0]][39:32] : trace_mem[rx_addr[2:0]][7:0]);
                                    rsp_buf2 <= (ui_in[0] ? trace_mem[rx_addr[2:0]][47:40] : trace_mem[rx_addr[2:0]][15:8]);
                                    rsp_buf3 <= (ui_in[0] ? trace_mem[rx_addr[2:0]][55:48] : trace_mem[rx_addr[2:0]][23:16]);
                                    rsp_buf4 <= (ui_in[0] ? trace_mem[rx_addr[2:0]][63:56] : trace_mem[rx_addr[2:0]][31:24]);
                                    rsp_index <= 3'd0;
                                    rsp_valid <= 1'b1;
                                end

                                OP_FAULT: begin
                                    fault_force_error <= rx_data[0];
                                    fault_timeout     <= rx_data[1];
                                    fault_delay       <= rx_data[11:8];
                                    rsp_buf0 <= ST_OK;
                                    rsp_buf1 <= {4'b0, rx_data[11:8]};
                                    rsp_buf2 <= 8'd0;
                                    rsp_buf3 <= 8'd0;
                                    rsp_buf4 <= 8'd0;
                                    rsp_index <= 3'd0;
                                    rsp_valid <= 1'b1;
                                end

                                OP_CLEAR: begin
                                    error_flags   <= 8'd0;
                                    error_count   <= 8'd0;
                                    timeout_count <= 8'd0;
                                    protocol_count <= 8'd0;
                                    rsp_buf0 <= ST_OK;
                                    rsp_buf1 <= 8'd0;
                                    rsp_buf2 <= 8'd0;
                                    rsp_buf3 <= 8'd0;
                                    rsp_buf4 <= 8'd0;
                                    rsp_index <= 3'd0;
                                    rsp_valid <= 1'b1;
                                end

                                default: begin
                                    error_flags[0] <= 1'b1; // bad opcode
                                    rsp_buf0 <= ST_BADOP;
                                    rsp_buf1 <= rx_op;
                                    rsp_buf2 <= 8'd0;
                                    rsp_buf3 <= 8'd0;
                                    rsp_buf4 <= 8'd0;
                                    rsp_index <= 3'd0;
                                    rsp_valid <= 1'b1;
                                end
                            endcase
                        end
                        default: begin
                            rx_active <= 1'b0;
                            error_flags[3] <= 1'b1;
                        end
                    endcase
                end
            end

            // Transaction completion / bookkeeping.
            if (eng_done) begin
                rsp_buf0 <= eng_status;
                rsp_buf1 <= eng_rdata[7:0];
                rsp_buf2 <= eng_rdata[15:8];
                rsp_buf3 <= eng_rdata[23:16];
                rsp_buf4 <= eng_rdata[31:24];
                rsp_index <= 3'd0;
                rsp_valid <= 1'b1;

                total_count <= total_count + 8'd1;
                if (eng_write)
                    write_count <= write_count + 8'd1;
                else
                    read_count <= read_count + 8'd1;

                if (eng_status != ST_OK)
                    error_count <= error_count + 8'd1;
                if (eng_timeout)
                    timeout_count <= timeout_count + 8'd1;
                if (eng_protocol_error)
                    protocol_count <= protocol_count + 8'd1;
                if (eng_latency > max_latency)
                    max_latency <= eng_latency;

                if (eng_status == ST_TIMEOUT)
                    error_flags[1] <= 1'b1;
                if (eng_status == ST_PROTOCOL)
                    error_flags[2] <= 1'b1;

                trace_mem[trace_wr_ptr] <= {
                    cycle_counter[10:0],
                    eng_trace_meta,
                    eng_trace_status,
                    eng_trace_addr,
                    eng_trace_data
                };
                trace_wr_ptr <= trace_wr_ptr + 3'd1;
            end
        end
    end

    // Keep the formal top-level signal list warning-free.
    wire _unused = &{1'b0, uio_in[7:6]};

endmodule

// ============================================================================
// Protocol transaction engine
// ============================================================================
module pnx_bus_engine (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    input  wire        apb_sel,
    input  wire        write,
    input  wire [15:0] addr,
    input  wire [31:0] wdata,
    input  wire [3:0]  wstrb,
    input  wire        fault_force_error,
    input  wire        fault_timeout,
    input  wire [3:0]  fault_delay,
    output wire        busy,
    output reg         done,
    output reg [7:0]   status,
    output reg [31:0]  rdata,
    output reg [15:0]  trace_addr,
    output reg [31:0]  trace_data,
    output reg [2:0]   trace_meta,
    output reg [1:0]   trace_status,
    output reg [5:0]   latency,
    output reg         protocol_error,
    output reg         timeout_seen,
    output reg         rf_wr_en,
    output reg [15:0]  rf_wr_addr,
    output reg [31:0]  rf_wr_data,
    output reg [3:0]   rf_wr_strb,
    output reg [15:0]  rf_rd_addr,
    input  wire [31:0] rf_rd_data
);

    localparam [2:0] S_IDLE   = 3'd0;
    localparam [2:0] S_DELAY  = 3'd1;
    localparam [2:0] S_AXISND = 3'd2;
    localparam [2:0] S_AXIRSP = 3'd3;
    localparam [2:0] S_APBSET = 3'd4;
    localparam [2:0] S_APBACC = 3'd5;
    localparam [2:0] S_DONE   = 3'd6;

    localparam [5:0] TIMEOUT_MAX = 6'd31;

    reg [2:0] state;
    reg       req_apb, req_write;
    reg [15:0] req_addr;
    reg [31:0] req_wdata;
    reg [3:0] req_wstrb;
    reg       cfg_force_error;
    reg       cfg_timeout;
    reg [3:0] delay_cnt;
    reg [5:0] timeout_cnt;

    // AXI4-Lite channel state.
    reg awvalid, wvalid, bvalid, arvalid, rvalid;
    reg awready, wready, bready, arready, rready;
    reg [1:0] bresp, rresp;
    reg [31:0] rresp_data;

    // APB4 state.
    reg psel, penable, pready, pslverr;
    reg [31:0] prdata;

    assign busy = (state != S_IDLE);

    wire addr_aligned = (req_addr[1:0] == 2'b00);
    wire addr_in_range = (req_addr[15:7] == 9'd0);

    // Internal target behavior. The target never waits unless fault_timeout is set.
    always @* begin
        awready = (!cfg_timeout && (state == S_AXISND));
        wready  = (!cfg_timeout && (state == S_AXISND));
        bready  = 1'b1;
        arready = (!cfg_timeout && (state == S_AXISND));
        rready  = 1'b1;

        pready  = (!cfg_timeout && (state == S_APBACC));
        pslverr = cfg_force_error || !addr_in_range || !addr_aligned;
        prdata  = rf_rd_data;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            req_apb <= 1'b0;
            req_write <= 1'b0;
            req_addr <= 16'd0;
            req_wdata <= 32'd0;
            req_wstrb <= 4'hF;
            cfg_force_error <= 1'b0;
            cfg_timeout <= 1'b0;
            delay_cnt <= 4'd0;
            timeout_cnt <= 6'd0;
            done <= 1'b0;
            status <= 8'h00;
            rdata <= 32'd0;
            trace_addr <= 16'd0;
            trace_data <= 32'd0;
            trace_meta <= 3'd0;
            trace_status <= 2'd0;
            latency <= 6'd0;
            protocol_error <= 1'b0;
            timeout_seen <= 1'b0;
            rf_wr_en <= 1'b0;
            rf_wr_addr <= 16'd0;
            rf_wr_data <= 32'd0;
            rf_wr_strb <= 4'd0;
            rf_rd_addr <= 16'd0;
            awvalid <= 1'b0;
            wvalid <= 1'b0;
            bvalid <= 1'b0;
            arvalid <= 1'b0;
            rvalid <= 1'b0;
            bresp <= 2'b00;
            rresp <= 2'b00;
            rresp_data <= 32'd0;
            psel <= 1'b0;
            penable <= 1'b0;
        end else begin
            done <= 1'b0;
            rf_wr_en <= 1'b0;

            // Protocol checker: APB ACCESS is illegal without PSEL.
            if (penable && !psel)
                protocol_error <= 1'b1;

            if (state != S_IDLE)
                latency <= latency + 6'd1;

            case (state)
                S_IDLE: begin
                    latency <= 6'd0;
                    timeout_cnt <= 6'd0;
                    protocol_error <= 1'b0;
                    timeout_seen <= 1'b0;
                    if (start) begin
                        req_apb <= apb_sel;
                        req_write <= write;
                        req_addr <= addr;
                        req_wdata <= wdata;
                        req_wstrb <= wstrb;
                        cfg_force_error <= fault_force_error;
                        cfg_timeout <= fault_timeout;
                        delay_cnt <= fault_delay;
                        rf_rd_addr <= addr;
                        state <= S_DELAY;
                    end
                end

                S_DELAY: begin
                    if (delay_cnt != 0) begin
                        delay_cnt <= delay_cnt - 4'd1;
                    end else if (req_apb) begin
                        psel <= 1'b1;
                        penable <= 1'b0;
                        state <= S_APBSET;
                    end else begin
                        awvalid <= req_write;
                        wvalid <= req_write;
                        arvalid <= ~req_write;
                        bvalid <= 1'b0;
                        rvalid <= 1'b0;
                        state <= S_AXISND;
                    end
                end

                S_AXISND: begin
                    // AXI AW/W are independent VALID channels. This master presents
                    // both for writes and waits until both handshakes are complete.
                    if (req_write) begin
                        if (awvalid && awready)
                            awvalid <= 1'b0;
                        if (wvalid && wready)
                            wvalid <= 1'b0;

                        if ((!awvalid || awready) && (!wvalid || wready)) begin
                            if (cfg_timeout) begin
                                // Keep waiting; timeout logic below will terminate.
                                bvalid <= 1'b0;
                            end else begin
                                if (cfg_force_error || !addr_in_range || !addr_aligned) begin
                                    bresp <= 2'b10; // SLVERR
                                end else begin
                                    bresp <= 2'b00; // OKAY
                                    rf_wr_en <= 1'b1;
                                    rf_wr_addr <= req_addr;
                                    rf_wr_data <= req_wdata;
                                    rf_wr_strb <= req_wstrb;
                                end
                                bvalid <= 1'b1;
                                state <= S_AXIRSP;
                            end
                        end
                    end else begin
                        if (arvalid && arready)
                            arvalid <= 1'b0;
                        if (arvalid && arready) begin
                            if (cfg_timeout) begin
                                rvalid <= 1'b0;
                            end else begin
                                rresp_data <= rf_rd_data;
                                if (cfg_force_error || !addr_in_range || !addr_aligned)
                                    rresp <= 2'b10;
                                else
                                    rresp <= 2'b00;
                                rvalid <= 1'b1;
                                state <= S_AXIRSP;
                            end
                        end
                    end

                    if ((awvalid || wvalid || arvalid) && !awready && !wready && !arready) begin
                        if (timeout_cnt == TIMEOUT_MAX) begin
                            timeout_seen <= 1'b1;
                            status <= 8'h02;
                            trace_status <= 2'b10;
                            trace_addr <= req_addr;
                            trace_data <= req_write ? req_wdata : 32'd0;
                            trace_meta <= req_apb ? (req_write ? 3'b011 : 3'b010) : (req_write ? 3'b001 : 3'b000);
                            state <= S_DONE;
                        end else begin
                            timeout_cnt <= timeout_cnt + 6'd1;
                        end
                    end
                end

                S_AXIRSP: begin
                    if (req_write) begin
                        if (bvalid && bready) begin
                            status <= (bresp == 2'b00) ? 8'h00 : 8'h01;
                            rdata <= 32'd0;
                            trace_addr <= req_addr;
                            trace_data <= req_wdata;
                            trace_meta <= 3'b001;
                            trace_status <= (bresp == 2'b00) ? 2'b00 : 2'b01;
                            bvalid <= 1'b0;
                            state <= S_DONE;
                        end
                    end else begin
                        if (rvalid && rready) begin
                            status <= (rresp == 2'b00) ? 8'h00 : 8'h01;
                            rdata <= rresp_data;
                            trace_addr <= req_addr;
                            trace_data <= rresp_data;
                            trace_meta <= 3'b000;
                            trace_status <= (rresp == 2'b00) ? 2'b00 : 2'b01;
                            rvalid <= 1'b0;
                            state <= S_DONE;
                        end
                    end

                    if (timeout_cnt == TIMEOUT_MAX) begin
                        timeout_seen <= 1'b1;
                        status <= 8'h02;
                        rdata <= 32'd0;
                        trace_addr <= req_addr;
                        trace_data <= req_write ? req_wdata : 32'd0;
                        trace_meta <= req_write ? 3'b001 : 3'b000;
                        trace_status <= 2'b10;
                        state <= S_DONE;
                    end else if (bvalid || rvalid) begin
                        timeout_cnt <= 6'd0;
                    end else begin
                        timeout_cnt <= timeout_cnt + 6'd1;
                    end
                end

                S_APBSET: begin
                    // APB requires SETUP (PSEL=1, PENABLE=0) for one cycle before ACCESS.
                    if (penable) begin
                        protocol_error <= 1'b1;
                        status <= 8'h03;
                        trace_status <= 2'b11;
                        state <= S_DONE;
                    end else begin
                        penable <= 1'b1;
                        state <= S_APBACC;
                    end
                end

                S_APBACC: begin
                    if (psel && penable && pready) begin
                        if (pslverr) begin
                            status <= 8'h01;
                            rdata <= 32'd0;
                            trace_status <= 2'b01;
                        end else begin
                            status <= 8'h00;
                            if (req_write) begin
                                rf_wr_en <= 1'b1;
                                rf_wr_addr <= req_addr;
                                rf_wr_data <= req_wdata;
                                rf_wr_strb <= req_wstrb;
                                rdata <= 32'd0;
                            end else begin
                                rdata <= prdata;
                            end
                            trace_status <= 2'b00;
                        end
                        trace_addr <= req_addr;
                        trace_data <= req_write ? req_wdata : prdata;
                        trace_meta <= req_write ? 3'b011 : 3'b010;
                        psel <= 1'b0;
                        penable <= 1'b0;
                        state <= S_DONE;
                    end else if (timeout_cnt == TIMEOUT_MAX) begin
                        timeout_seen <= 1'b1;
                        status <= 8'h02;
                        rdata <= 32'd0;
                        trace_addr <= req_addr;
                        trace_data <= req_write ? req_wdata : 32'd0;
                        trace_meta <= req_write ? 3'b011 : 3'b010;
                        trace_status <= 2'b10;
                        psel <= 1'b0;
                        penable <= 1'b0;
                        state <= S_DONE;
                    end else begin
                        timeout_cnt <= timeout_cnt + 6'd1;
                    end
                end

                S_DONE: begin
                    done <= 1'b1;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
