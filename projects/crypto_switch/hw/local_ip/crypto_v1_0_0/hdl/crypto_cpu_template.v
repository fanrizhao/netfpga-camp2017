`include "crypto_cpu_regs_defines.v"

//parameters to be added to the top module parameters
#(
    // AXI Registers Data Width
    parameter C_S_AXI_DATA_WIDTH    = 32,
    parameter C_S_AXI_ADDR_WIDTH    = 32
)
//ports to be added to the top module ports
(
// Signals for AXI_IP and IF_REG (Added for debug purposes)
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
)


    // define and assign default little endian
    wire                                      reg_key_default_little;
    assign  reg_key_default_little = `REG_KEY_DEFAULT;

    // define registers
    reg      [`REG_ID_BITS]    id_reg;
    reg      [`REG_VERSION_BITS]    version_reg;
    wire     [`REG_RESET_BITS]    reset_reg;
    reg      [`REG_FLIP_BITS]    ip2cpu_flip_reg;
    wire     [`REG_FLIP_BITS]    cpu2ip_flip_reg;
    reg      [`REG_DEBUG_BITS]    ip2cpu_debug_reg;
    wire     [`REG_DEBUG_BITS]    cpu2ip_debug_reg;
    reg      [`REG_PKTIN_BITS]    pktin_reg;
    wire                             pktin_reg_clear;
    reg      [`REG_PKTOUT_BITS]    pktout_reg;
    wire                             pktout_reg_clear;
    wire     [`REG_KEY_BITS]    key_reg;

//Registers section
 crypto_cpu_regs
 #(
     .C_BASE_ADDRESS        (C_BASEADDR ),
     .C_S_AXI_DATA_WIDTH    (C_S_AXI_DATA_WIDTH),
     .C_S_AXI_ADDR_WIDTH    (C_S_AXI_ADDR_WIDTH)
 ) crypto_cpu_regs_inst
 (
   // General ports
    .clk                    (axis_aclk),
    .resetn                 (axis_resetn),
   // AXI Lite ports
    .S_AXI_ACLK             (S_AXI_ACLK),
    .S_AXI_ARESETN          (S_AXI_ARESETN),
    .S_AXI_AWADDR           (S_AXI_AWADDR),
    .S_AXI_AWVALID          (S_AXI_AWVALID),
    .S_AXI_WDATA            (S_AXI_WDATA),
    .S_AXI_WSTRB            (S_AXI_WSTRB),
    .S_AXI_WVALID           (S_AXI_WVALID),
    .S_AXI_BREADY           (S_AXI_BREADY),
    .S_AXI_ARADDR           (S_AXI_ARADDR),
    .S_AXI_ARVALID          (S_AXI_ARVALID),
    .S_AXI_RREADY           (S_AXI_RREADY),
    .S_AXI_ARREADY          (S_AXI_ARREADY),
    .S_AXI_RDATA            (S_AXI_RDATA),
    .S_AXI_RRESP            (S_AXI_RRESP),
    .S_AXI_RVALID           (S_AXI_RVALID),
    .S_AXI_WREADY           (S_AXI_WREADY),
    .S_AXI_BRESP            (S_AXI_BRESP),
    .S_AXI_BVALID           (S_AXI_BVALID),
    .S_AXI_AWREADY          (S_AXI_AWREADY),

   // Register ports
   .id_reg          (id_reg),
   .version_reg          (version_reg),
   .reset_reg          (reset_reg),
   .ip2cpu_flip_reg          (ip2cpu_flip_reg),
   .cpu2ip_flip_reg          (cpu2ip_flip_reg),
   .ip2cpu_debug_reg          (ip2cpu_debug_reg),
   .cpu2ip_debug_reg          (cpu2ip_debug_reg),
   .pktin_reg          (pktin_reg),
   .pktin_reg_clear    (pktin_reg_clear),
   .pktout_reg          (pktout_reg),
   .pktout_reg_clear    (pktout_reg_clear),
   .key_reg          (key_reg),
   // Global Registers - user can select if to use
   .cpu_resetn_soft(),//software reset, after cpu module
   .resetn_soft    (),//software reset to cpu module (from central reset management)
   .resetn_sync    (resetn_sync)//synchronized reset, use for better timing
);
//registers logic, current logic is just a placeholder for initial compil, required to be changed by the user
always @(posedge axis_aclk)
	if (~resetn_sync) begin
        id_reg <= #1    `REG_ID_DEFAULT;
        version_reg <= #1    `REG_VERSION_DEFAULT;
        ip2cpu_flip_reg <= #1    `REG_FLIP_DEFAULT;
        ip2cpu_debug_reg <= #1    `REG_DEBUG_DEFAULT;
        pktin_reg <= #1    `REG_PKTIN_DEFAULT;
        pktout_reg <= #1    `REG_PKTOUT_DEFAULT;
	end
	else begin
        id_reg <= #1    `REG_ID_DEFAULT;
        version_reg <= #1    `REG_VERSION_DEFAULT;
		ip2cpu_flip_reg <= #1 cpu2ip_flip_reg;
		ip2cpu_debug_reg <= #1 cpu2ip_debug_reg;
        pktin_reg <= #1 pktin_reg_clear ? 'h0  : `REG_PKTIN_DEFAULT;
        pktout_reg <= #1 pktout_reg_clear ? 'h0  : `REG_PKTOUT_DEFAULT;
        end

