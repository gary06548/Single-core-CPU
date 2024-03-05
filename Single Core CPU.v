module CPU(

		clk,
	      rst_n,
  
	   IO_stall,

         awid_m_inf,
       awaddr_m_inf,
       awsize_m_inf,
      awburst_m_inf,
        awlen_m_inf,
      awvalid_m_inf,
      awready_m_inf,
                    
        wdata_m_inf,
        wlast_m_inf,
       wvalid_m_inf,
       wready_m_inf,
                    
          bid_m_inf,
        bresp_m_inf,
       bvalid_m_inf,
       bready_m_inf,
                    
         arid_m_inf,
       araddr_m_inf,
        arlen_m_inf,
       arsize_m_inf,
      arburst_m_inf,
      arvalid_m_inf,
                    
      arready_m_inf, 
          rid_m_inf,
        rdata_m_inf,
        rresp_m_inf,
        rlast_m_inf,
       rvalid_m_inf,
       rready_m_inf 

);
// Input port
input  wire clk, rst_n;
// Output port
output reg  IO_stall;

parameter ID_WIDTH = 4 , ADDR_WIDTH = 32, DATA_WIDTH = 16, DRAM_NUMBER=2, WRIT_NUMBER=1;

// AXI Interface wire connecttion for pseudo DRAM read/write

// axi write address channel 
output  wire [WRIT_NUMBER * ID_WIDTH-1:0]        awid_m_inf;
output  wire [WRIT_NUMBER * ADDR_WIDTH-1:0]    awaddr_m_inf;
output  wire [WRIT_NUMBER * 3 -1:0]            awsize_m_inf;
output  wire [WRIT_NUMBER * 2 -1:0]           awburst_m_inf;
output  wire [WRIT_NUMBER * 7 -1:0]             awlen_m_inf;
output  wire [WRIT_NUMBER-1:0]                awvalid_m_inf;
input   wire [WRIT_NUMBER-1:0]                awready_m_inf;
// axi write data channel 
output  wire [WRIT_NUMBER * DATA_WIDTH-1:0]     wdata_m_inf;
output  wire [WRIT_NUMBER-1:0]                  wlast_m_inf;
output  wire [WRIT_NUMBER-1:0]                 wvalid_m_inf;
input   wire [WRIT_NUMBER-1:0]                 wready_m_inf;
// axi write response channel
input   wire [WRIT_NUMBER * ID_WIDTH-1:0]         bid_m_inf;
input   wire [WRIT_NUMBER * 2 -1:0]             bresp_m_inf;
input   wire [WRIT_NUMBER-1:0]             	   bvalid_m_inf;
output  wire [WRIT_NUMBER-1:0]                 bready_m_inf;
// -----------------------------
// axi read address channel 
output  wire [DRAM_NUMBER * ID_WIDTH-1:0]       arid_m_inf;
output  wire [DRAM_NUMBER * ADDR_WIDTH-1:0]   araddr_m_inf;
output  wire [DRAM_NUMBER * 7 -1:0]            arlen_m_inf;
output  wire [DRAM_NUMBER * 3 -1:0]           arsize_m_inf;
output  wire [DRAM_NUMBER * 2 -1:0]          arburst_m_inf;
output  wire [DRAM_NUMBER-1:0]               arvalid_m_inf;
input   wire [DRAM_NUMBER-1:0]               arready_m_inf;
// -----------------------------
// axi read data channel 
input   wire [DRAM_NUMBER * ID_WIDTH-1:0]         rid_m_inf;
input   wire [DRAM_NUMBER * DATA_WIDTH-1:0]     rdata_m_inf;
input   wire [DRAM_NUMBER * 2 -1:0]             rresp_m_inf;
input   wire [DRAM_NUMBER-1:0]                  rlast_m_inf;
input   wire [DRAM_NUMBER-1:0]                 rvalid_m_inf;
output  wire [DRAM_NUMBER-1:0]                 rready_m_inf;
// -----------------------------

//###########################################
//      Wrtie down your design below       //
//###########################################
localparam signed OFFSET = 13'h1000;
localparam IDLE         = 3'd0;
localparam INST_FETCH   = 3'd1;
localparam INST_DECODE  = 3'd2;
localparam EXECUTE      = 3'd3;
localparam MEM_LOAD     = 3'd4;
localparam MEM_STORE    = 3'd5;
localparam WRITE_BACK   = 3'd6;

//####################################################
//               reg & wire
//####################################################
// registers of CPU
reg signed [15:0] core_r0 , core_r1 , core_r2 , core_r3 ;
reg signed [15:0] core_r4 , core_r5 , core_r6 , core_r7 ;
reg signed [15:0] core_r8 , core_r9 , core_r10, core_r11;
reg signed [15:0] core_r12, core_r13, core_r14, core_r15;

// state register
reg [2:0] c_state, n_state;

// element of instruction
wire [15:0] inst, data;
wire [12:0] addr  = inst[12:0];
wire [2:0] opcode = inst[15:13];
wire [3:0] rs_idx = inst[12:9];
wire [3:0] rt_idx = inst[8:5];
wire [3:0] rd_idx = inst[4:1];
wire func         = inst[0];
wire signed [4:0] imm = inst[4:0];

// control signal of getting instruction
reg  inst_req;      // send request for getting machine code
wire inst_valid;    // get instruction code successfully

// control signal of reading data
reg  r_data_req;    // send request for reading data
wire r_data_valid;  // read data successfully

// control signal of writing data
reg  w_data_req;    // send request for writting data
wire w_data_done;   // write data successfully

// program counter
reg signed [11:0] PC;

// register
reg signed [15:0] rs_data, rt_data, rd_data;

// accessing address of data memory 
wire signed [11:0] data_addr;

//###########################################
//                   FSM                   //
//###########################################
/*
ADD   : INST_FETCH -> INST_DECODE -> EXECUTE -> WRITE_BACK -> INST_FETCH
SUB   : INST_FETCH -> INST_DECODE -> EXECUTE -> WRITE_BACK -> INST_FETCH
SLT   : INST_FETCH -> INST_DECODE -> EXECUTE -> WRITE_BACK -> INST_FETCH
Mult  : INST_FETCH -> INST_DECODE -> EXECUTE -> WRITE_BACK -> INST_FETCH
Load  : INST_FETCH -> INST_DECODE -> EXECUTE -> MEM_LOAD   -> INST_FETCH
Store : INST_FETCH -> INST_DECODE -> EXECUTE -> MEM_STORE  -> INST_FETCH
BEQ   : INST_FETCH -> INST_DECODE -> EXECUTE -> INST_FETCH
Jump  : INST_FETCH -> INST_DECODE -> EXECUTE -> INST_FETCH
P.S. LOAD doesn't need WRITE_BACK
*/

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) c_state <= IDLE;
    else        c_state <= n_state;
end

always @* begin
    case (c_state) 
        IDLE: begin
            n_state = INST_FETCH;
        end
        INST_FETCH: begin
            if (inst_valid) n_state = INST_DECODE;
            else            n_state = INST_FETCH;
        end
        INST_DECODE: begin
            n_state = EXECUTE;
        end
        EXECUTE: begin
            case (opcode)
                3'b000:  n_state = WRITE_BACK;
                3'b001:  n_state = WRITE_BACK;
                3'b010:  n_state = MEM_LOAD;
                3'b011:  n_state = MEM_STORE;
                3'b100:  n_state = INST_FETCH;
                3'b101:  n_state = INST_FETCH;
                default: n_state = INST_FETCH;
            endcase
        end
        MEM_LOAD: begin
            if (r_data_valid) n_state = INST_FETCH;
            else              n_state = MEM_LOAD;
        end
        MEM_STORE: begin
            if (w_data_done)  n_state = INST_FETCH;
            else              n_state = MEM_STORE;
        end
        WRITE_BACK: begin
            n_state = INST_FETCH;
        end
        default: begin
            n_state = IDLE;
        end
    endcase
end

//================================================================
//                         INST_DECODE                          //
//================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) rs_data <= 0 ;
    else begin
        if (n_state == INST_DECODE) begin
            case(rs_idx)
                0:  rs_data <= core_r0;
                1:  rs_data <= core_r1;
                2:  rs_data <= core_r2;
                3:  rs_data <= core_r3;
                4:  rs_data <= core_r4;
                5:  rs_data <= core_r5;
                6:  rs_data <= core_r6;
                7:  rs_data <= core_r7;
                8:  rs_data <= core_r8;
                9:  rs_data <= core_r9;
                10: rs_data <= core_r10; 
                11: rs_data <= core_r11; 
                12: rs_data <= core_r12; 
                13: rs_data <= core_r13; 
                14: rs_data <= core_r14; 
                15: rs_data <= core_r15; 
            endcase
        end
        else begin
            rs_data <= rs_data;
        end
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) rt_data <= 0 ;
    else begin
        if (n_state == INST_DECODE) begin
            case(rt_idx)
                0:  rt_data <= core_r0;
                1:  rt_data <= core_r1;
                2:  rt_data <= core_r2;
                3:  rt_data <= core_r3;
                4:  rt_data <= core_r4;
                5:  rt_data <= core_r5;
                6:  rt_data <= core_r6;
                7:  rt_data <= core_r7;
                8:  rt_data <= core_r8;
                9:  rt_data <= core_r9;
                10: rt_data <= core_r10; 
                11: rt_data <= core_r11; 
                12: rt_data <= core_r12; 
                13: rt_data <= core_r13; 
                14: rt_data <= core_r14; 
                15: rt_data <= core_r15; 
            endcase
        end
        else begin
            rt_data <= rt_data;
        end    
    end
end

//================================================================
//                           EXECUTE                            //
//================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) PC <= 12'd0;
    else begin
        if (n_state == EXECUTE) begin
            if (opcode == 3'd5) PC <= addr;
            else if (opcode[2]) begin
                if (rs_data == rt_data) PC <= PC + 2 + (imm << 1);
                else                    PC <= PC + 2;
            end
            else PC <= PC + 2;
        end
        else PC <= PC;
    end
end

reg signed [15:0] result_1; // add, sub, set less than
reg signed [15:0] result_2; // mul

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) result_1 <= 0;
    else begin
        case ({opcode[0], func})
            2'b00: result_1 <= rs_data + rt_data; // ADD
            2'b01: result_1 <= rs_data - rt_data; // SUB
            2'b10: result_1 <= rs_data < rt_data; // Set Less Than
            2'b11: result_1 <= result_1;          // mul -> use result_2 to store
        endcase
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) result_2 <= 0;
    else        result_2 <= rs_data * rt_data;
end

always @(*) begin
    if (opcode[0] && func) rd_data = result_2; // mult
    else                   rd_data = result_1; // add, sub, set less than
end

//================================================================
//   CORE_REG
//================================================================
wire n_state_is_WRITE_BACK = (n_state == WRITE_BACK);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)   core_r0 <= 0 ;
    else begin 
        if (n_state_is_WRITE_BACK && rd_idx == 4'd0)   core_r0 <= rd_data;
        else if (r_data_valid && rt_idx == 4'd0)       core_r0 <= data;
        else                                           core_r0 <= core_r0;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)   core_r1 <= 0 ;
    else begin
        if (n_state_is_WRITE_BACK && rd_idx == 4'd1)   core_r1 <= rd_data;
        else if (r_data_valid && rt_idx == 4'd1)       core_r1 <= data;
        else                                           core_r1 <= core_r1;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)   core_r2 <= 0 ;
    else begin
        if (n_state_is_WRITE_BACK && rd_idx == 4'd2)   core_r2 <= rd_data;
        else if (r_data_valid && rt_idx == 4'd2)       core_r2 <= data;
        else                                           core_r2 <= core_r2;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)   core_r3 <= 0 ;
    else begin
        if (n_state_is_WRITE_BACK && rd_idx == 4'd3)   core_r3 <= rd_data;
        else if (r_data_valid && rt_idx == 4'd3)       core_r3 <= data;
        else                                           core_r3 <= core_r3;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)   core_r4 <= 0 ;
    else begin
        if (n_state_is_WRITE_BACK && rd_idx == 4'd4)   core_r4 <= rd_data;
        else if (r_data_valid && rt_idx == 4'd4)       core_r4 <= data;
        else                                           core_r4 <= core_r4;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)   core_r5 <= 0 ;
    else begin
        if (n_state_is_WRITE_BACK && rd_idx == 4'd5)   core_r5 <= rd_data;
        else if (r_data_valid && rt_idx == 4'd5)       core_r5 <= data;
        else                                           core_r5 <= core_r5;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)   core_r6 <= 0 ;
    else begin
        if (n_state_is_WRITE_BACK && rd_idx == 4'd6)   core_r6 <= rd_data;
        else if (r_data_valid && rt_idx == 4'd6)       core_r6 <= data;
        else                                           core_r6 <= core_r6;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)   core_r7 <= 0 ;
    else begin
        if (n_state_is_WRITE_BACK && rd_idx == 4'd7)   core_r7 <= rd_data;
        else if (r_data_valid && rt_idx == 4'd7)       core_r7 <= data;
        else                                           core_r7 <= core_r7;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)   core_r8 <= 0 ;
    else begin
        if (n_state_is_WRITE_BACK && rd_idx == 4'd8)   core_r8 <= rd_data;
        else if (r_data_valid && rt_idx == 4'd8)       core_r8 <= data;
        else                                           core_r8 <= core_r8;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)   core_r9 <= 0 ;
    else begin
        if (n_state_is_WRITE_BACK && rd_idx == 4'd9)   core_r9 <= rd_data;
        else if (r_data_valid && rt_idx == 4'd9)       core_r9 <= data;
        else                                           core_r9 <= core_r9;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)   core_r10 <= 0 ;
    else begin
        if (n_state_is_WRITE_BACK && rd_idx == 4'd10)  core_r10 <= rd_data;
        else if (r_data_valid && rt_idx == 4'd10)      core_r10 <= data;
        else                                           core_r10 <= core_r10;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)   core_r11 <= 0 ;
    else begin
        if (n_state_is_WRITE_BACK && rd_idx == 4'd11)  core_r11 <= rd_data;
        else if (r_data_valid && rt_idx == 4'd11)      core_r11 <= data;
        else                                           core_r11 <= core_r11;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)   core_r12 <= 0 ;
    else begin
        if (n_state_is_WRITE_BACK && rd_idx == 4'd12)  core_r12 <= rd_data;
        else if (r_data_valid && rt_idx == 4'd12)      core_r12 <= data;
        else                                           core_r12 <= core_r12;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)   core_r13 <= 0 ;
    else begin
        if (n_state_is_WRITE_BACK && rd_idx == 4'd13)  core_r13 <= rd_data;
        else if (r_data_valid && rt_idx == 4'd13)      core_r13 <= data;
        else                                           core_r13 <= core_r13;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)   core_r14 <= 0 ;
    else begin
        if (n_state_is_WRITE_BACK && rd_idx == 4'd14)  core_r14 <= rd_data;
        else if (r_data_valid && rt_idx == 4'd14)      core_r14 <= data;
        else                                           core_r14 <= core_r14;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)   core_r15 <= 0 ;
    else begin
        if (n_state_is_WRITE_BACK && rd_idx == 4'd15)  core_r15 <= rd_data;
        else if (r_data_valid && rt_idx == 4'd15)      core_r15 <= data;
        else                                           core_r15 <= core_r15;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)   IO_stall <= 1'b1;
    else begin
        if (n_state == INST_FETCH && c_state != INST_FETCH && c_state != IDLE)  
            IO_stall <= 1'b0;
        else
            IO_stall <= 1'b1;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)   
        inst_req <= 1'b0;
    else 
        inst_req <= (c_state != INST_FETCH && n_state == INST_FETCH);
end

BRIDGE_INST_R #(.ID_WIDTH(ID_WIDTH), .ADDR_WIDTH(ADDR_WIDTH), .DATA_WIDTH(DATA_WIDTH)) INST_MEM_R(
    // Input Signals
    .clk(clk), .rst_n(rst_n), .request(inst_req), .addr(PC[11:1]),

    // Output signals
    .r_valid(inst_valid), .r_inst(inst),
    
    // DRAM Signals
    .arid_m_inf(arid_m_inf[7:4]), 
    .arvalid_m_inf(arvalid_m_inf[1]), 
    .araddr_m_inf(araddr_m_inf[63:32]), 
    .arready_m_inf(arready_m_inf[1]), 
    .arlen_m_inf(arlen_m_inf[13:7]), 
    .arburst_m_inf(arburst_m_inf[3:2]), 
    .arsize_m_inf(arsize_m_inf[5:3]), 
    .rid_m_inf(rid_m_inf[7:4]), 
    .rready_m_inf(rready_m_inf[1]), 
    .rvalid_m_inf(rvalid_m_inf[1]), 
    .rresp_m_inf(rresp_m_inf[3:2]), 
    .rdata_m_inf(rdata_m_inf[31:16]), 
    .rlast_m_inf(rlast_m_inf[1])
);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)   
        r_data_req <= 1'b0;
    else 
        r_data_req <= (c_state == EXECUTE && (n_state == MEM_LOAD || n_state == MEM_STORE));
end

assign data_addr = ((rs_data + imm) << 1);// + OFFSET;

BRIDGE_DATA_R #(.ID_WIDTH(ID_WIDTH), .ADDR_WIDTH(ADDR_WIDTH), .DATA_WIDTH(DATA_WIDTH)) DATA_MEM_R(
    // Input Signals
    .clk(clk), .rst_n(rst_n), .request(r_data_req), .addr(data_addr[11:1]), .rt_data(rt_data), .is_writting_DRAM(c_state == MEM_STORE),

    // Output signals
    .r_valid(r_data_valid), .r_data(data),
    
    // DRAM Signals
    .arid_m_inf(arid_m_inf[3:0]), 
    .arvalid_m_inf(arvalid_m_inf[0]), 
    .araddr_m_inf(araddr_m_inf[31:0]), 
    .arready_m_inf(arready_m_inf[0]), 
    .arlen_m_inf(arlen_m_inf[6:0]), 
    .arburst_m_inf(arburst_m_inf[1:0]), 
    .arsize_m_inf(arsize_m_inf[2:0]), 
    .rid_m_inf(rid_m_inf[3:0]), 
    .rready_m_inf(rready_m_inf[0]), 
    .rvalid_m_inf(rvalid_m_inf[0]), 
    .rresp_m_inf(rresp_m_inf[1:0]), 
    .rdata_m_inf(rdata_m_inf[15:0]), 
    .rlast_m_inf(rlast_m_inf[0])
);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)   
        w_data_req <= 1'b0;
    else 
        w_data_req <= (c_state == EXECUTE && n_state == MEM_STORE);
end

BRIDGE_DATA_W #(.ID_WIDTH(ID_WIDTH), .ADDR_WIDTH(ADDR_WIDTH), .DATA_WIDTH(DATA_WIDTH)) DATA_MEM_W(
    // Input Signals
    .clk(clk), .rst_n(rst_n), .request(w_data_req), .addr(data_addr[11:1]), .rt_data(rt_data),

    // Output signals
    .w_done(w_data_done),
    
    // DRAM Signals
    .awid_m_inf(awid_m_inf), 
    .awsize_m_inf(awsize_m_inf), 
    .awburst_m_inf(awburst_m_inf), 
    .awlen_m_inf(awlen_m_inf),
    .awvalid_m_inf(awvalid_m_inf), 
    .awaddr_m_inf(awaddr_m_inf), 
    .awready_m_inf(awready_m_inf),
    .wvalid_m_inf(wvalid_m_inf), 
    .wready_m_inf(wready_m_inf), 
    .wdata_m_inf(wdata_m_inf), 
    .wlast_m_inf(wlast_m_inf),
    .bid_m_inf(bid_m_inf), 
    .bready_m_inf(bready_m_inf), 
    .bvalid_m_inf(bvalid_m_inf), 
    .bresp_m_inf(bresp_m_inf)
);

endmodule

module BRIDGE_INST_R #(parameter ID_WIDTH = 4 , ADDR_WIDTH = 32, DATA_WIDTH = 16) (
    // Input Signals
    clk, rst_n, request, addr,

    // Output signals
    r_valid, r_inst,
    
    // DRAM Signals
    arid_m_inf, arvalid_m_inf, araddr_m_inf, arready_m_inf, arlen_m_inf, arburst_m_inf, arsize_m_inf, 
    rid_m_inf, rready_m_inf, rvalid_m_inf, rresp_m_inf, rdata_m_inf, rlast_m_inf
);

// Input Signals
input clk, rst_n, request;
input [10:0] addr; // LSB of address must be zero, so use 11 bits to store it

// Output signals
output reg r_valid;
output reg [DATA_WIDTH-1:0] r_inst;

// ------------------------
// <<<<< AXI READ >>>>>
// ------------------------
// (1)	axi read address channel 
output wire [ID_WIDTH-1:0]      arid_m_inf;
output reg                   arvalid_m_inf;
input  wire                  arready_m_inf;
output reg [ADDR_WIDTH-1:0]   araddr_m_inf;
output wire [6:0]              arlen_m_inf;
output wire [2:0]             arsize_m_inf;
output wire [1:0]            arburst_m_inf;
// ------------------------
// (2)	axi read data channel 
input  wire [ID_WIDTH-1:0]       rid_m_inf;
input  wire                   rvalid_m_inf;
output reg                    rready_m_inf;
input  wire [DATA_WIDTH-1:0]   rdata_m_inf;
input  wire                    rlast_m_inf;
input  wire [1:0]              rresp_m_inf;

//==============================================//
//                  reg / wire                  //
//==============================================//
reg [2:0] c_state, n_state;
reg [15:0] sram_out;
reg [6:0] sram_addr;
reg [3:0] tag;
wire [15:0] sram_in;
wire WEB;

//----- DRAM read constant -----
assign arid_m_inf    = 4'd0; 	// id = 0
assign arburst_m_inf = 2'b01;	// incr mode
assign arsize_m_inf  = 3'b001;	// size = 2 bytes 
assign arlen_m_inf   = 7'd127;

always @(posedge clk) begin
    araddr_m_inf <= {20'd1, addr[10:7], 8'd0};
end

//==============================================//
//                     FSM                      //
//==============================================//
localparam Idle         = 3'd0;
localparam Wait_DRAM_r1 = 3'd1;
localparam Wait_DRAM_r2 = 3'd2;
localparam Wait_SRAM_r1 = 3'd3; // hit -> wait 2 cycles to get data
localparam Wait_SRAM_r2 = 3'd4; // hit -> wait 2 cycles to get data
localparam Output       = 3'd5; 
localparam Wait_SRAM_r3 = 3'd6; // hit -> wait 2 cycles to get data

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) c_state <= Idle;
    else        c_state <= n_state;
end

always @* begin
  	case (c_state)
  		Idle: begin
            if (request) begin
                if (tag == addr[10:7])
                    n_state = Wait_SRAM_r1;
                else
                    n_state = Wait_DRAM_r1;
            end        
    		else    n_state = Idle;
        end
  		Wait_DRAM_r1: begin
            if (arready_m_inf)
                  n_state = Wait_DRAM_r2;
            else
                  n_state = Wait_DRAM_r1;
  		end
        Wait_DRAM_r2: begin
            if (rvalid_m_inf) begin 
                if (rlast_m_inf) n_state = Output;
                else             n_state = Wait_DRAM_r2;
            end
            else                 n_state = Wait_DRAM_r2;
  		end
        Wait_SRAM_r1: n_state = Wait_SRAM_r2;
        Wait_SRAM_r2: n_state = Wait_SRAM_r3;
        Wait_SRAM_r3: n_state = Output;
        Output:       n_state = Idle;
  		default:      n_state = Idle;
  	endcase
end

//==============================================//
//                   Design                     //
//==============================================//
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) sram_addr <= 7'd0;
    else begin
        if (n_state == Wait_SRAM_r1) sram_addr <= addr[6:0];
        else if (rvalid_m_inf)       sram_addr <= sram_addr + 1'b1;
        else if (n_state == Idle)    sram_addr <= 7'd0;
        else                         sram_addr <= sram_addr;
    end
end

assign WEB = (c_state != Wait_DRAM_r2);
assign sram_in = (c_state == Idle)? 16'd0 : (c_state == Output)? 16'hffff : rdata_m_inf;

SRAM_128x16 SRAM_inst( .A0(sram_addr[0]),   .A1(sram_addr[1]),   .A2(sram_addr[2]),   .A3(sram_addr[3]), 
                       .A4(sram_addr[4]),   .A5(sram_addr[5]),   .A6(sram_addr[6]), 
                       .DO0(sram_out[0]),   .DO1(sram_out[1]),   .DO2(sram_out[2]),   .DO3(sram_out[3]),
                       .DO4(sram_out[4]),   .DO5(sram_out[5]),   .DO6(sram_out[6]),   .DO7(sram_out[7]),
                       .DO8(sram_out[8]),   .DO9(sram_out[9]),   .DO10(sram_out[10]), .DO11(sram_out[11]),
                       .DO12(sram_out[12]), .DO13(sram_out[13]), .DO14(sram_out[14]), .DO15(sram_out[15]),
                       .DI0(sram_in[0]),    .DI1(sram_in[1]),   .DI2(sram_in[2]),   .DI3(sram_in[3]),
                       .DI4(sram_in[4]),    .DI5(sram_in[5]),   .DI6(sram_in[6]),   .DI7(sram_in[7]),
                       .DI8(sram_in[8]),    .DI9(sram_in[9]),   .DI10(sram_in[10]), .DI11(sram_in[11]),
                       .DI12(sram_in[12]),  .DI13(sram_in[13]), .DI14(sram_in[14]), .DI15(sram_in[15]),
                       .CK(clk), .WEB(WEB), .OE(1'b1), .CS(1'b1));

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) tag <= 4'hf;
    else begin
        if (c_state == Wait_DRAM_r1)  
            tag <= addr[10:7];
        else
            tag <= tag;
    end 
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        arvalid_m_inf <= 1'b0;
    end
    else begin
        case (c_state) 
            Wait_DRAM_r1: begin
                if (arready_m_inf) begin 
                    arvalid_m_inf <= 1'b0;
                end
                else begin
                    arvalid_m_inf <= 1'b1;
                end
            end
            default: begin
                arvalid_m_inf <= 1'b0;
            end
        endcase
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rready_m_inf <= 1'b0;
    end
    else begin
        case (c_state) 
            Wait_DRAM_r2: begin
                if (rlast_m_inf) rready_m_inf <= 1'b0;
                else             rready_m_inf <= 1'b1;
            end
            default: begin
                rready_m_inf <= rready_m_inf;
            end
        endcase
    end
end

//==============================================//
//                 Output                       //
//==============================================//
reg [15:0] sram_out_tmp;
always @(posedge clk) begin
    sram_out_tmp <= sram_out;
end

always @(posedge clk) begin
    if (c_state == Wait_SRAM_r3) begin //c_state == Wait_SRAM_r2
        //r_inst <= sram_out;
        r_inst <= sram_out_tmp;
    end
    else if (c_state == Wait_DRAM_r2 && rvalid_m_inf && sram_addr == addr[6:0]) begin
        r_inst <= rdata_m_inf;
    end
    else begin
        r_inst <= r_inst;
    end    
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        r_valid <= 1'b0;
    else 
        r_valid <= (n_state == Output); // r_valid <= (n_state == Output) || (c_state == Wait_SRAM_r2);
end

endmodule

module BRIDGE_DATA_R #(parameter ID_WIDTH = 4 , ADDR_WIDTH = 32, DATA_WIDTH = 16) (
    // Input Signals
    clk, rst_n, request, addr, rt_data, is_writting_DRAM,

    // Output signals
    r_valid, r_data,
    
    // DRAM Signals
    arid_m_inf, arvalid_m_inf, araddr_m_inf, arready_m_inf, arlen_m_inf, arburst_m_inf, arsize_m_inf, 
    rid_m_inf, rready_m_inf, rvalid_m_inf, rresp_m_inf, rdata_m_inf, rlast_m_inf
);

// Input Signals
input clk, rst_n, request, is_writting_DRAM;
input [10:0] addr; // LSB of address must be zero, so use 11 bits to store it
input [15:0] rt_data;

// Output signals
output reg r_valid;
output reg [DATA_WIDTH-1:0] r_data;

// ------------------------
// <<<<< AXI READ >>>>>
// ------------------------
// (1)	axi read address channel 
output wire [ID_WIDTH-1:0]      arid_m_inf;
output reg                   arvalid_m_inf;
input  wire                  arready_m_inf;
output reg [ADDR_WIDTH-1:0]   araddr_m_inf;
output wire [6:0]              arlen_m_inf;
output wire [2:0]             arsize_m_inf;
output wire [1:0]            arburst_m_inf;
// ------------------------
// (2)	axi read data channel 
input  wire [ID_WIDTH-1:0]       rid_m_inf;
input  wire                   rvalid_m_inf;
output reg                    rready_m_inf;
input  wire [DATA_WIDTH-1:0]   rdata_m_inf;
input  wire                    rlast_m_inf;
input  wire [1:0]              rresp_m_inf;

//==============================================//
//                  reg / wire                  //
//==============================================//
reg [2:0] c_state, n_state;
reg [15:0] sram_out;
reg [6:0] sram_addr;
reg [3:0] tag;
wire [15:0] sram_in;
wire WEB;

//----- DRAM read constant -----
assign arid_m_inf    = 4'd0; 	// id = 0
assign arburst_m_inf = 2'b01;	// incr mode
assign arsize_m_inf  = 3'b001;	// size = 2 bytes 
assign arlen_m_inf   = 7'd127;

always @(posedge clk) begin
    araddr_m_inf <= {20'd1, addr[10:7], 8'd0};
end

//==============================================//
//                     FSM                      //
//==============================================//
localparam Idle         = 3'd0;
localparam Wait_DRAM_r1 = 3'd1;
localparam Wait_DRAM_r2 = 3'd2;
localparam Wait_SRAM_r1 = 3'd3; // hit -> wait 2 cycles to get data
localparam Wait_SRAM_r2 = 3'd4; // hit -> wait 2 cycles to get data
localparam Output       = 3'd5; 
localparam Write_Thru   = 3'd6; // update data of SRAM when writting SRAM
localparam Wait_SRAM_r3 = 3'd7; // hit -> wait 2 cycles to get data

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) c_state <= Idle;
    else        c_state <= n_state;
end

always @* begin
  	case (c_state)
  		Idle: begin
            if (request) begin
                if (tag == addr[10:7]) begin // hit
                    if (is_writting_DRAM) n_state = Write_Thru;   // update the data in SRAM 
                    else                  n_state = Wait_SRAM_r1; // read the data from SRAM
                end                    
                else begin // miss
                    if (is_writting_DRAM) n_state = Idle;         // can't read and write at the same time, or there will be data racing issue
                    else                  n_state = Wait_DRAM_r1; 
                end
            end        
    		else  n_state = Idle;
        end
  		Wait_DRAM_r1: begin
            if (arready_m_inf) n_state = Wait_DRAM_r2;
            else               n_state = Wait_DRAM_r1;
  		end
        Wait_DRAM_r2: begin
            if (rvalid_m_inf) begin 
                if (rlast_m_inf) n_state = Output;
                else             n_state = Wait_DRAM_r2;
            end
            else                 n_state = Wait_DRAM_r2;
  		end
        Wait_SRAM_r1: n_state = Wait_SRAM_r2;
        Wait_SRAM_r2: n_state = Wait_SRAM_r3;
        Wait_SRAM_r3: n_state = Output;
        Output:       n_state = Idle;
        Write_Thru:   n_state = Idle;
  		default:      n_state = Idle;
  	endcase
end

//==============================================//
//                   Design                     //
//==============================================//
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)     sram_addr <= 7'd0;
    else begin
        if (rvalid_m_inf)                                          sram_addr <= sram_addr + 1'b1;
        else if (n_state == Wait_SRAM_r1 || n_state == Write_Thru) sram_addr <= addr[6:0];
        else if (n_state == Idle)                                  sram_addr <= 7'd0;
        else                                                       sram_addr <= sram_addr;
    end
end

assign WEB = (c_state != Wait_DRAM_r2 && c_state != Write_Thru);
assign sram_in = (c_state == Idle)? 16'd0 : (c_state == Write_Thru)? rt_data : rdata_m_inf;

SRAM_128x16 SRAM_data( .A0(sram_addr[0]),   .A1(sram_addr[1]),   .A2(sram_addr[2]),   .A3(sram_addr[3]), 
                       .A4(sram_addr[4]),   .A5(sram_addr[5]),   .A6(sram_addr[6]), 
                       .DO0(sram_out[0]),   .DO1(sram_out[1]),   .DO2(sram_out[2]),   .DO3(sram_out[3]),
                       .DO4(sram_out[4]),   .DO5(sram_out[5]),   .DO6(sram_out[6]),   .DO7(sram_out[7]),
                       .DO8(sram_out[8]),   .DO9(sram_out[9]),   .DO10(sram_out[10]), .DO11(sram_out[11]),
                       .DO12(sram_out[12]), .DO13(sram_out[13]), .DO14(sram_out[14]), .DO15(sram_out[15]),
                       .DI0(sram_in[0]),    .DI1(sram_in[1]),   .DI2(sram_in[2]),   .DI3(sram_in[3]),
                       .DI4(sram_in[4]),    .DI5(sram_in[5]),   .DI6(sram_in[6]),   .DI7(sram_in[7]),
                       .DI8(sram_in[8]),    .DI9(sram_in[9]),   .DI10(sram_in[10]), .DI11(sram_in[11]),
                       .DI12(sram_in[12]),  .DI13(sram_in[13]), .DI14(sram_in[14]), .DI15(sram_in[15]),
                       .CK(clk), .WEB(WEB), .OE(1'b1), .CS(1'b1));

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) tag <= 4'hf;
    else begin
        if (c_state == Wait_DRAM_r1)  
            tag <= addr[10:7];
        else
            tag <= tag;
    end 
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        arvalid_m_inf <= 1'b0;
    end
    else begin
        case (c_state) 
            Wait_DRAM_r1: begin
                if (arready_m_inf) begin 
                    arvalid_m_inf <= 1'b0;
                end
                else begin
                    arvalid_m_inf <= 1'b1;
                end
            end
            default: begin
                arvalid_m_inf <= 1'b0;
            end
        endcase
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rready_m_inf <= 1'b0;
    end
    else begin
        case (c_state) 
            Wait_DRAM_r2: begin
                if (rlast_m_inf) rready_m_inf <= 1'b0;
                else             rready_m_inf <= 1'b1;
            end
            default: begin
                rready_m_inf <= rready_m_inf;
            end
        endcase
    end
end

//==============================================//
//                 Output                       //
//==============================================//
reg [15:0] sram_out_tmp;
always @(posedge clk) begin
    sram_out_tmp <= sram_out;
end

always @(posedge clk) begin
    if (c_state == Wait_SRAM_r3) begin 
        r_data <= sram_out_tmp;
    end
    else if (c_state == Wait_DRAM_r2 && rvalid_m_inf && sram_addr == addr[6:0]) begin
        r_data <= rdata_m_inf;
    end
    else begin
        r_data <= r_data;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        r_valid <= 1'b0;
    else 
        r_valid <= (n_state == Output);
end

endmodule


module BRIDGE_DATA_W #(parameter ID_WIDTH = 4 , ADDR_WIDTH = 32, DATA_WIDTH = 16) (
    // Input Signals
    clk, rst_n, request, addr, rt_data,

    // Output signals
    w_done,
    
    // DRAM Signals
    awid_m_inf, awsize_m_inf, awburst_m_inf, awlen_m_inf,
    awvalid_m_inf, awaddr_m_inf, awready_m_inf,
    wvalid_m_inf, wready_m_inf, wdata_m_inf, wlast_m_inf,
    bid_m_inf, bready_m_inf, bvalid_m_inf, bresp_m_inf
);

// Input Signals
input clk, rst_n, request;
input [10:0] addr;
input [DATA_WIDTH-1:0] rt_data;

// Output Signal
output reg w_done;

// ------------------------
// <<<<< AXI WRITE >>>>>
// ------------------------
// (1) 	axi write address channel 
output wire [ID_WIDTH-1:0]      awid_m_inf;
output wire [2:0]             awsize_m_inf;
output wire [1:0]            awburst_m_inf;
output wire [6:0]              awlen_m_inf;
output reg                   awvalid_m_inf;
output reg [ADDR_WIDTH-1:0]   awaddr_m_inf;
input  wire                  awready_m_inf;

// -------------------------
// (2)	axi write data channel 
output reg                    wvalid_m_inf;
input  wire                   wready_m_inf;
output wire [DATA_WIDTH-1:0]   wdata_m_inf;
output wire                    wlast_m_inf;
// -------------------------
// (3)	axi write response channel 
input  wire [ID_WIDTH-1:0]       bid_m_inf;
output reg                    bready_m_inf;
input  wire                   bvalid_m_inf;
input  wire  [1:0]             bresp_m_inf;
// -----------------------------

// DRAM write constant
assign awid_m_inf = 0;
assign awlen_m_inf = 7'd0;
assign awsize_m_inf = 3'b001;
assign awburst_m_inf = 2'b01;

assign wdata_m_inf = rt_data;

always @(posedge clk) begin
    awaddr_m_inf <= {20'd1, addr, 1'b0};
end

//==============================================//
//                     FSM                      //
//==============================================//
reg [1:0] c_state, n_state;

localparam Idle         = 2'd0;
localparam Wait_DRAM_w1 = 2'd1;
localparam Wait_DRAM_w2 = 2'd2; // wait wready and write dram
localparam Wait_DRAM_w3 = 2'd3; // wait bresp

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) c_state <= Idle;
    else        c_state <= n_state;
end

always @* begin
  	case (c_state)
  		Idle: begin
            if (request)        n_state = Wait_DRAM_w1;
    		else                n_state = Idle;
        end
  		Wait_DRAM_w1: begin
            if (awready_m_inf)  n_state = Wait_DRAM_w2;
            else                n_state = Wait_DRAM_w1;
  		end
  		Wait_DRAM_w2: begin
            if (wready_m_inf)
                n_state = Wait_DRAM_w3;
            else                
                n_state = Wait_DRAM_w2;
  		end
        Wait_DRAM_w3: begin
            if (bvalid_m_inf)   n_state = Idle;
            else                n_state = Wait_DRAM_w3;
        end
  	endcase
end


//==============================================//
//                   Design                     //
//==============================================//
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        awvalid_m_inf <= 1'b0;
    end
   else begin
        case (c_state) 
            Wait_DRAM_w1: begin
                if (awready_m_inf) begin
                    awvalid_m_inf <= 1'b0;
                end
                else begin
                    awvalid_m_inf <= 1'b1;
                end
            end
            default: begin
                awvalid_m_inf <= 1'b0;
            end
        endcase
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) bready_m_inf <= 1'b0;
    else begin
        if (c_state == Wait_DRAM_w3) begin
            if (bvalid_m_inf) bready_m_inf <= 1'b0;
            else              bready_m_inf <= 1'b1;
        end
        else begin
            bready_m_inf <= 1'b0;
        end
    end  
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) wvalid_m_inf <= 1'b0;
    else begin
        if (c_state == Wait_DRAM_w2) begin
            wvalid_m_inf <= 1'b1;
        end
        else begin
            wvalid_m_inf <= 1'b0;
        end
    end  
end

assign wlast_m_inf = wvalid_m_inf;

//==============================================//
//                   Output                     //
//==============================================//
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) w_done <= 1'b0;
    else begin
        if (c_state == Wait_DRAM_w3 && n_state == Idle)    
            w_done <= 1'b1;
        else                    
            w_done <= 1'b0;
    end
end

endmodule
