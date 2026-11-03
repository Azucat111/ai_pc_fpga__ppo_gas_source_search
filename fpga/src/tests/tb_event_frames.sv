`timescale 1ns/1ps
module tb_event_frames;
 reg clk=0;always #5 clk=~clk;
 reg rstn=0,bv=0;reg[7:0]data=0;wire mv,ev;wire[255:0]command;wire[31:0]errors;
 event_frame_rx dut(clk,rstn,1'b0,bv,data,mv,command,errors,ev);
 integer motions=0,events=0,i;reg[7:0]p[0:31];reg[15:0]crc;
 function[15:0]update(input[15:0]x,input[7:0]v);reg[15:0]r;integer k;begin r=x^{v,8'b0};for(k=0;k<8;k=k+1)r=r[15]?(r<<1)^16'h1021:r<<1;update=r;end endfunction
 always @(posedge clk)begin if(mv)motions=motions+1;if(ev)events=events+1;if(mv && ev)$fatal(1,"cross-dispatch");end
 task send(input[7:0]kind,version,input bad_crc);
 begin
  for(i=0;i<32;i=i+1)p[i]=0;
  p[0]=8'ha5;p[1]=8'h5a;p[2]=version;p[3]=kind;p[4]=1;
  crc=16'hffff;for(i=0;i<30;i=i+1)crc=update(crc,p[i]);
  p[30]=crc[7:0];p[31]=crc[15:8]^bad_crc;
  for(i=0;i<32;i=i+1)begin @(negedge clk);bv=1;data=p[i];@(negedge clk);bv=0;end
  repeat(3)@(negedge clk);
 end endtask
 initial begin
  repeat(3)@(negedge clk);rstn=1;
  send(1,1,0);send(2,1,0);
  if(motions!=1 || events!=1)$fatal(1,"type dispatch failed");
  send(2,1,1);send(2,2,0);send(99,1,0);
  if(motions!=1 || events!=1 || errors!=3)$fatal(1,"invalid events escaped CRC/version/type filter");
  $display("PASS event framing: independent type1/type2 dispatch; invalid CRC/version/type rejected without event or motor command");$finish;
 end
endmodule
