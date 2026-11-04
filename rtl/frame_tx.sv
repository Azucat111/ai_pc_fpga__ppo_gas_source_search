`timescale 1ns/1ps
module frame_tx #(parameter [7:0] FRAME_TYPE=8'h81)(input wire clk,rstn,start,input wire [1439:0] payload,
 input wire [31:0] sequence_number,input wire byte_ready,
 output wire byte_valid,output reg [7:0] byte_data,output wire busy);
 function automatic [15:0] crc_byte(input [15:0] c,input [7:0] b);
   reg [15:0] x;integer i;begin x=c^{b,8'b0};
   for(i=0;i<8;i=i+1) x=x[15]?(x<<1)^16'h1021:x<<1;
   crc_byte=x;end
 endfunction
 reg [1519:0] buffer;
 reg [15:0] crc;
 integer index;
 assign busy=index<192;
 assign byte_valid=busy;
 always @* begin
   if(index<190) byte_data=buffer[index*8+:8];
   else if(index==190) byte_data=crc[7:0];else byte_data=crc[15:8];
 end
 always @(posedge clk) begin
   if(!rstn) begin index<=192;buffer<=0;crc<=16'hffff;end
   else if(!busy && start) begin
     buffer<={16'b0,payload,sequence_number,FRAME_TYPE,8'h01,8'h5a,8'ha5};
     index<=0;crc<=16'hffff;
   end else if(busy && byte_ready) begin
     if(index<190) crc<=crc_byte(crc,byte_data);
     index<=index+1;
   end
 end
endmodule
