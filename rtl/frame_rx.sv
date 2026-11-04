`timescale 1ns/1ps
module frame_rx(input wire clk,rstn,ms_tick,byte_valid,input wire [7:0] byte_data,
 output reg command_valid,output reg [255:0] command,
 output reg [31:0] crc_errors);
 function automatic [15:0] crc_byte(input [15:0] c,input [7:0] b);
   reg [15:0] x;integer i;begin x=c^{b,8'b0};
   for(i=0;i<8;i=i+1) x=x[15]?(x<<1)^16'h1021:x<<1;
   crc_byte=x;end
 endfunction
 reg [255:0] buffer;
 reg [15:0] crc;
 integer index,idle_ms;
 always @(posedge clk) begin
   if(!rstn) begin index<=0;idle_ms<=0;crc<=16'hffff;buffer<=0;command<=0;command_valid<=0;crc_errors<=0;end
   else begin
     command_valid<=0;
     if(ms_tick && index!=0) begin
       if(idle_ms>=4) begin index<=0;idle_ms<=0;end
       else idle_ms<=idle_ms+1;
     end
     if(byte_valid) begin
       idle_ms<=0;
       if(index==0) begin
         if(byte_data==8'ha5) begin buffer[7:0]<=byte_data;index<=1;crc<=crc_byte(16'hffff,byte_data);end
       end else if(index==1) begin
         if(byte_data==8'h5a) begin buffer[15:8]<=byte_data;index<=2;crc<=crc_byte(crc,byte_data);end
         else if(byte_data==8'ha5) crc<=crc_byte(16'hffff,byte_data);
         else index<=0;
       end else begin
         buffer[index*8+:8]<=byte_data;
         if(index<30) crc<=crc_byte(crc,byte_data);
         if(index==31) begin
           index<=0;
           if({byte_data,buffer[247:240]}==crc && buffer[23:16]==1 && buffer[31:24]==1 &&
              buffer[143:136]==0 && buffer[239:224]==0) begin
             command<= {byte_data,buffer[247:0]};command_valid<=1;
           end else crc_errors<=crc_errors+1;
         end else index<=index+1;
       end
     end
   end
 end
endmodule
