//-
// Copyright (c) 2015 University of Cambridge
// Copyright (c) 2015 Noa Zilberman
// All rights reserved.
//
// This software was developed by the University of Cambridge Computer Laboratory 
// under EPSRC INTERNET Project EP/H040536/1, National Science Foundation under Grant No. CNS-0855268,
// and Defense Advanced Research Projects Agency (DARPA) and Air Force Research Laboratory (AFRL), 
// under contract FA8750-11-C-0249.
//
// @NETFPGA_LICENSE_HEADER_START@
//
// Licensed to NetFPGA Open Systems C.I.C. (NetFPGA) under one or more contributor
// license agreements.  See the NOTICE file distributed with this work for
// additional information regarding copyright ownership.  NetFPGA licenses this
// file to you under the NetFPGA Hardware-Software License, Version 1.0 (the
// "License"); you may not use this file except in compliance with the
// License.  You may obtain a copy of the License at:
//
//   http://www.netfpga-cic.org
//
// Unless required by applicable law or agreed to in writing, Work distributed
// under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied.  See the License for the
// specific language governing permissions and limitations under the License.
//
// @NETFPGA_LICENSE_HEADER_END@
//



//`include "my_module_cpu_regs_defines.v"

module my_module
#(
    //Master AXI Stream Data Width
    parameter C_M_AXIS_DATA_WIDTH=256,
    parameter C_S_AXIS_DATA_WIDTH=256,
    parameter C_M_AXIS_TUSER_WIDTH=128,
    parameter C_S_AXIS_TUSER_WIDTH=128,
    parameter SRC_PORT_POS=16,
    parameter DST_PORT_POS=24,

 // AXI Registers Data Width
    parameter C_S_AXI_DATA_WIDTH    = 32,          
    parameter C_S_AXI_ADDR_WIDTH    = 12,          
    parameter C_USE_WSTRB           = 0,	   
    parameter C_DPHASE_TIMEOUT      = 0,               
    parameter C_NUM_ADDRESS_RANGES = 1,
    parameter  C_TOTAL_NUM_CE       = 1,
    parameter  C_S_AXI_MIN_SIZE    = 32'h0000_FFFF,
    //parameter [0:32*2*C_NUM_ADDRESS_RANGES-1]   C_ARD_ADDR_RANGE_ARRAY  = 
    //                                             {2*C_NUM_ADDRESS_RANGES
    //                                              {32'h00000000}
    //                                             },
    parameter [0:8*C_NUM_ADDRESS_RANGES-1] C_ARD_NUM_CE_ARRAY  = 
                                                {
                                                 C_NUM_ADDRESS_RANGES{8'd1}
                                                 },
    parameter     C_FAMILY            = "virtex7", 
    parameter C_BASEADDR            = 32'h00000000,
    parameter C_HIGHADDR            = 32'h0000FFFF


)
(
    // Global Ports
    input axis_aclk,
    input axis_resetn,

    // Master Stream Ports (interface to data path)
    output reg [C_M_AXIS_DATA_WIDTH - 1:0] m_axis_tdata,
    output reg [((C_M_AXIS_DATA_WIDTH / 8)) - 1:0] m_axis_tkeep,
    output reg [C_M_AXIS_TUSER_WIDTH-1:0] m_axis_tuser,
    output reg m_axis_tvalid,
    input   m_axis_tready,
    output reg m_axis_tlast,

    // Slave Stream Ports (interface to RX queues)
    input [C_S_AXIS_DATA_WIDTH - 1:0] s_axis_tdata,
    input [((C_S_AXIS_DATA_WIDTH / 8)) - 1:0] s_axis_tkeep,
    input [C_S_AXIS_TUSER_WIDTH-1:0] s_axis_tuser,
    input  s_axis_tvalid,
    output s_axis_tready,
    input  s_axis_tlast,

// Slave AXI Ports
    input                                     S_AXI_ACLK,
    input                                     S_AXI_ARESETN,
    input      [C_S_AXI_ADDR_WIDTH-1 : 0]     S_AXI_AWADDR,
    input                                     S_AXI_AWVALID,
    input      [C_S_AXI_DATA_WIDTH-1 : 0]     S_AXI_WDATA,
    input      [C_S_AXI_DATA_WIDTH/8-1 : 0]   S_AXI_WSTRB,
    input                                     S_AXI_WVALID,
    input                                     S_AXI_BREADY,
    input      [C_S_AXI_ADDR_WIDTH-1 : 0]     S_AXI_ARADDR,
    input                                     S_AXI_ARVALID,
    input                                     S_AXI_RREADY,
    output                                    S_AXI_ARREADY,
    output     [C_S_AXI_DATA_WIDTH-1 : 0]     S_AXI_RDATA,
    output     [1 : 0]                        S_AXI_RRESP,
    output                                    S_AXI_RVALID,
    output                                    S_AXI_WREADY,
    output     [1 :0]                         S_AXI_BRESP,
    output                                    S_AXI_BVALID,
    output                                    S_AXI_AWREADY
);



   // ---------- Local Parameters ---------
   localparam LISTEN      	= 1;
   localparam CHECKIP       	= 2;
   localparam CHECKPRO         	= 3; 
   localparam DISCARD         	= 4; 
   localparam FORWARD         	= 5; 


   // ------------- Regs/ wires -----------

   wire                             fifo_nearly_full;
   wire                             fifo_empty;
   reg                              fifo_rd_en;
   wire [C_M_AXIS_TUSER_WIDTH-1:0]  fifo_out_tuser;
   wire [C_M_AXIS_DATA_WIDTH-1:0]   fifo_out_tdata;
   wire [C_M_AXIS_DATA_WIDTH/8-1:0] fifo_out_tkeep;
   wire  	                    fifo_out_tlast;
   wire                             fifo_tvalid;
   wire                             fifo_tlast;

   reg  [2:0]                       state, next_state;
   wire [31:0]                      key;
   reg [7:0] src_port;
   reg [7:0] proto;

   // ------------ Modules -------------

   fallthrough_small_fifo
   #( .WIDTH(C_M_AXIS_DATA_WIDTH+C_M_AXIS_TUSER_WIDTH+C_M_AXIS_DATA_WIDTH/8+1),
      .MAX_DEPTH_BITS(2)
    )
    input_fifo
    ( // Outputs
      .dout                         ({fifo_out_tlast, fifo_out_tuser, fifo_out_tkeep, fifo_out_tdata}),
      .full                         (),
      .nearly_full                  (fifo_nearly_full),
      .prog_full                    (),
      .empty                        (fifo_empty),
      // Inputs
      .din                          ({s_axis_tlast, s_axis_tuser, s_axis_tkeep, s_axis_tdata}),
      .wr_en                        (s_axis_tvalid & s_axis_tready),
      .rd_en                        (fifo_rd_en),
      .reset                        (~axis_resetn),
      .clk                          (axis_aclk));

   // ------------- Logic ------------

   assign s_axis_tready = !fifo_nearly_full;
  /*********************************************************************
   * Wait until the ethernet header has been decoded and the output
   * port is found, then write the module header and move the packet
   * to the output
   **********************************************************************/
   always @* begin
     m_axis_tuser = 0;//fifo_out_tuser;
     m_axis_tdata = 0;//fifo_out_tdata;
     m_axis_tkeep = 0;//fifo_out_tkeep;
     m_axis_tlast = 0;//fifo_out_tlast;
     m_axis_tvalid = 0;
      
     fifo_rd_en = 0;
     
     next_state = state;
	 src_port = fifo_out_tuser[23:16];
	 proto = fifo_out_tdata[191:184];
	case(state)
		LISTEN: begin//{
			
			
			if (!fifo_empty && src_port == 8'b1 && proto == 8'd1  ) begin//{
    			m_axis_tvalid = !fifo_empty; 
    			
				//if(src_port &  8'h1) begin//{
				m_axis_tuser = fifo_out_tuser;
				m_axis_tuser[31:24] = src_port;
				m_axis_tdata = fifo_out_tdata;
				m_axis_tkeep = fifo_out_tkeep;
				m_axis_tlast = fifo_out_tlast;
				if( m_axis_tvalid && m_axis_tready) begin//{
					
					next_state = FORWARD;
					fifo_rd_en = 1; 
					end//}
				else 
					next_state = LISTEN;
				
				end//}
			else if (!fifo_empty && src_port == 8'b1 && (proto == 8'd6 || proto == 8'd17)  ) begin//{
    			m_axis_tvalid = !fifo_empty; 
    			
				//if(src_port &  8'h1) begin//{
				m_axis_tuser = fifo_out_tuser;
				m_axis_tuser[31:24] = 8'd4;
				m_axis_tdata = fifo_out_tdata;
				m_axis_tkeep = fifo_out_tkeep;
				m_axis_tlast = fifo_out_tlast;
				if( m_axis_tvalid && m_axis_tready) begin//{
					
					next_state = FORWARD;
					fifo_rd_en = 1; 
					end//}
				else 
					next_state = LISTEN;
				
				end//}
		    else if (!fifo_empty && src_port != 8'b1 )
		   		next_state = DISCARD;
		   	else 	
		   	    next_state = LISTEN;
			end//}

		FORWARD: begin
		    m_axis_tvalid = !fifo_empty; 
		    m_axis_tuser = fifo_out_tuser;
			m_axis_tdata = fifo_out_tdata;
			m_axis_tkeep = fifo_out_tkeep;
			m_axis_tlast = fifo_out_tlast;
			if (m_axis_tvalid && m_axis_tready) begin
				fifo_rd_en = 1;
				
				if (fifo_out_tlast) 
					next_state = LISTEN;
				else 
					next_state = FORWARD;
			end
			else
				next_state = FORWARD;

			
		end 

		DISCARD: begin
			if (!fifo_empty) begin
				fifo_rd_en = 1;
				m_axis_tvalid = 0; 
				if (fifo_out_tlast) 
					next_state = LISTEN;
				else
					next_state = DISCARD;
			end 
			else
				next_state = DISCARD;
		end
	endcase 
   end

   always @(posedge axis_aclk) begin
     if (~axis_resetn) begin
       state <= LISTEN;
     end
     else begin
       state <= next_state;
     end
   end


//Registers section





endmodule // my_module
