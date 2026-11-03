`timescale 1ns/1ps
// Independent AW/W capture; one outstanding response on each AXI-Lite channel.
module car_axi #(
 parameter integer C_S_AXI_DATA_WIDTH=32,C_S_AXI_ADDR_WIDTH=12,CLK_HZ=5000000
)(
 (* X_INTERFACE_PARAMETER="ASSOCIATED_BUSIF S_AXI, ASSOCIATED_RESET s_axi_aresetn, FREQ_HZ 5000000" *)
 (* X_INTERFACE_INFO="xilinx.com:signal:clock:1.0 s_axi_aclk CLK" *) input wire s_axi_aclk,
 (* X_INTERFACE_PARAMETER="POLARITY ACTIVE_LOW" *)
 (* X_INTERFACE_INFO="xilinx.com:signal:reset:1.0 s_axi_aresetn RST" *) input wire s_axi_aresetn,
 (* X_INTERFACE_PARAMETER="XIL_INTERFACENAME S_AXI, PROTOCOL AXI4LITE, DATA_WIDTH 32, ADDR_WIDTH 12, READ_WRITE_MODE READ_WRITE" *)
 (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI AWADDR" *) input wire [C_S_AXI_ADDR_WIDTH-1:0] s_axi_awaddr,(* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI AWPROT" *) input wire [2:0] s_axi_awprot,
 (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI AWVALID" *) input wire s_axi_awvalid,(* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI AWREADY" *) output wire s_axi_awready,
 (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI WDATA" *) input wire [31:0] s_axi_wdata,(* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI WSTRB" *) input wire [3:0] s_axi_wstrb,
 (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI WVALID" *) input wire s_axi_wvalid,(* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI WREADY" *) output wire s_axi_wready,
 (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI BRESP" *) output wire [1:0] s_axi_bresp,(* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI BVALID" *) output reg s_axi_bvalid,(* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI BREADY" *) input wire s_axi_bready,
 (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI ARADDR" *) input wire [C_S_AXI_ADDR_WIDTH-1:0] s_axi_araddr,(* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI ARPROT" *) input wire [2:0] s_axi_arprot,
 (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI ARVALID" *) input wire s_axi_arvalid,(* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI ARREADY" *) output wire s_axi_arready,
 (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI RDATA" *) output reg [31:0] s_axi_rdata,(* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI RRESP" *) output wire [1:0] s_axi_rresp,(* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI RVALID" *) output reg s_axi_rvalid,(* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI RREADY" *) input wire s_axi_rready,
 input wire enc_m1_a,enc_m1_b,enc_m2_a,enc_m2_b,esp_rx,
 output wire motor_a_in1,motor_a_in2,motor_b_in1,motor_b_in2,esp_tx,
 output wire source_led,
 inout wire i2c_scl,i2c_sda
);
 reg aw_pending,w_pending;
 reg [11:0] write_addr;
 reg [31:0] write_data;
 reg [3:0] write_strobe;
 wire write_enable=aw_pending && w_pending && !s_axi_bvalid;
 wire [31:0] read_data;
 assign s_axi_awready=!aw_pending && !s_axi_bvalid;
 assign s_axi_wready=!w_pending && !s_axi_bvalid;
 assign s_axi_arready=!s_axi_rvalid;
 assign s_axi_bresp=0;
 assign s_axi_rresp=0;
 always @(posedge s_axi_aclk) begin
   if(!s_axi_aresetn) begin
     aw_pending<=0;w_pending<=0;write_addr<=0;write_data<=0;write_strobe<=0;
     s_axi_bvalid<=0;s_axi_rvalid<=0;s_axi_rdata<=0;
   end else begin
     if(s_axi_awvalid && s_axi_awready) begin write_addr<=s_axi_awaddr;aw_pending<=1;end
     if(s_axi_wvalid && s_axi_wready) begin write_data<=s_axi_wdata;write_strobe<=s_axi_wstrb;w_pending<=1;end
     if(s_axi_bvalid && s_axi_bready) s_axi_bvalid<=0;
     if(write_enable) begin aw_pending<=0;w_pending<=0;s_axi_bvalid<=1;end
     if(s_axi_rvalid && s_axi_rready) s_axi_rvalid<=0;
     if(s_axi_arvalid && s_axi_arready) begin s_axi_rdata<=read_data;s_axi_rvalid<=1;end
   end
 end
 car_core #(.CLK_HZ(CLK_HZ)) core(
  .clk(s_axi_aclk),.rstn(s_axi_aresetn),.wr_en(write_enable),.wr_addr(write_addr),
  .wr_data(write_data),.wr_strb(write_strobe),.rd_addr(s_axi_araddr),.rd_data(read_data),
  .enc_m1_a(enc_m1_a),.enc_m1_b(enc_m1_b),.enc_m2_a(enc_m2_a),.enc_m2_b(enc_m2_b),
  .motor_a_in1(motor_a_in1),.motor_a_in2(motor_a_in2),.motor_b_in1(motor_b_in1),.motor_b_in2(motor_b_in2),
  .esp_rx(esp_rx),.esp_tx(esp_tx),.source_led(source_led),.i2c_scl(i2c_scl),.i2c_sda(i2c_sda));
endmodule
