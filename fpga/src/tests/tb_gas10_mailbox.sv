`timescale 1ns/1ps
module tb_gas10_mailbox;
 reg clk=0;always #5 clk=~clk;
 reg rstn=0,wr=0,slot=0,ready=0;reg [11:0]addr=0,rd=0;reg [31:0]data=0;
 wire [31:0]out;wire valid,busy,starting,mode;wire [7:0]b;
 gas10_mailbox dut(clk,rstn,wr,addr,rd,data,4'hf,32'h12345678,32'd999,slot,ready,out,valid,b,busy,starting,mode);
 task write(input [11:0]a,input [31:0]v);begin @(negedge clk);addr=a;data=v;wr=1;@(negedge clk);wr=0;end endtask
 integer i,fd,count=0;reg [7:0]bytes[0:191];reg [15:0]crc;
 function [15:0]update(input [15:0]c,input [7:0]d);reg [15:0]x;integer j;begin x=c^{d,8'b0};for(j=0;j<8;j=j+1)x=x[15]?(x<<1)^16'h1021:x<<1;update=x;end endfunction
 always @(posedge clk)if(valid && ready)begin bytes[count]=b;count=count+1;end
 initial begin
  repeat(4)@(negedge clk);rstn=1;
  for(i=0;i<44;i=i+1)write(12'h300+i*4,i);
  write(12'h3b0,1);rd=12'h3b8;#1;if(out!=1)$fatal(1,"commit count");
  write(12'h308,32'hdeadbeef);write(12'h3b0,1);rd=12'h3bc;#1;if(out!=1)$fatal(1,"pending overwrite accepted");
  slot=1;repeat(4)@(negedge clk);slot=0;
  if(!busy || count!=0)$fatal(1,"backpressure ignored");
  write(12'h3b0,1);rd=12'h3bc;#1;if(out!=2)$fatal(1,"in-flight overwrite accepted");
  ready=1;wait(!busy);@(negedge clk);
  if(count!=192 || bytes[3]!=8'h82)$fatal(1,"GAS10 frame header/size");
  if({bytes[11],bytes[10],bytes[9],bytes[8]}!=32'h12345678)$fatal(1,"boot not supplied by PL");
  if({bytes[15],bytes[14],bytes[13],bytes[12]}!=999)$fatal(1,"commit clock not supplied by PL");
  if({bytes[19],bytes[18],bytes[17],bytes[16]}!=2)$fatal(1,"staging mutation leaked into committed scan");
  crc=16'hffff;for(i=0;i<190;i=i+1)crc=update(crc,bytes[i]);
  if({bytes[191],bytes[190]}!=crc)$fatal(1,"GAS10 CRC");
  write(12'h3c0,1);if(!mode)$fatal(1,"mode status");
  $display("PASS GAS10 mailbox: atomic scan, boot/time ownership, pending/inflight rejection, backpressure, CRC");$finish;
 end
 initial begin #100000;$fatal(1,"timeout");end
endmodule
