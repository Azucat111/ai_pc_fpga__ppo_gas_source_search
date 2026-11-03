`timescale 1ns/1ps
// Independent PS sensor mailbox and frame; no motion-control inputs or writes.
module gas10_mailbox(input wire clk,rstn,wr_en,input wire [11:0] wr_addr,rd_addr,
 input wire [31:0] wr_data,input wire [3:0] wr_strb,
 input wire [31:0] boot_id,now_ms,input wire tx_slot,byte_ready,
 output reg [31:0] rd_data,output wire byte_valid,output wire [7:0] byte_data,
 output wire busy,starting,output reg mode_enabled);
 reg [31:0] staging[0:44];reg [1439:0] committed;
 reg pending;reg [31:0] commits,rejected,scan_step,scan_errors;
 integer i;
 assign starting=pending && tx_slot && !busy;
 frame_tx #(.FRAME_TYPE(8'h82)) serializer(clk,rstn,starting,committed,commits,byte_ready,byte_valid,byte_data,busy);
 always @* begin
  rd_data=0;
  if(rd_addr>=12'h300 && rd_addr<12'h3b4)rd_data=staging[(rd_addr-12'h300)>>2];
  if(rd_addr>=12'h400 && rd_addr<12'h4b4)rd_data=committed[(rd_addr-12'h400)*8+:32];
  case(rd_addr)
   12'h3b4:rd_data={30'b0,busy,pending};
   12'h3b8:rd_data=commits;
   12'h3bc:rd_data=rejected;
   12'h3c0:rd_data={31'b0,mode_enabled};
   12'h3c4:rd_data=scan_step;
   12'h3c8:rd_data=scan_errors;
  endcase
 end
 always @(posedge clk)begin
  if(!rstn)begin
   committed<=0;pending<=0;commits<=0;rejected<=0;mode_enabled<=0;scan_step<=0;scan_errors<=0;
   for(i=0;i<45;i=i+1)staging[i]<=0;
  end else begin
   if(starting)pending<=0;
   if(wr_en && wr_strb==15 && wr_addr[1:0]==0)begin
    if(wr_addr>=12'h300 && wr_addr<12'h3b0)staging[(wr_addr-12'h300)>>2]<=wr_data;
    case(wr_addr)
     12'h3b0:if(wr_data[0])begin
      if(pending || busy)rejected<=rejected+1;
      else begin
       for(i=0;i<45;i=i+1)committed[i*32+:32]<=staging[i];
       committed[0+:32]<=boot_id;committed[32+:32]<=now_ms;
       pending<=1;commits<=commits+1;
      end
     end
     12'h3c0:mode_enabled<=wr_data[0];
     12'h3c4:scan_step<=wr_data;
     12'h3c8:scan_errors<=wr_data;
    endcase
   end
  end
 end
endmodule
