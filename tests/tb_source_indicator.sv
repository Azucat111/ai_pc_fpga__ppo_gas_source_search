`timescale 1ns/1ps
module tb_source_indicator;
 reg clk=0;always #5 clk=~clk;
 reg rstn=0,tick=0,valid=0;reg[31:0]boot=7,now_ms=100;
 reg [255:0]req=0;wire confirmed,led,busy,starting,bv;wire[7:0]bd;wire[31:0]seq;
 source_indicator dut(clk,rstn,tick,boot,now_ms,valid,req,1'b0,1'b1,bv,bd,busy,starting,confirmed,led,seq);
 task send(input[31:0]s,input[7:0]e,input[31:0]b,input[31:0]issued);
 begin @(negedge clk);req=0;req[63:32]=s;req[95:64]=b;req[127:96]=issued;req[135:128]=e;req[159:144]=500;valid=1;
 @(negedge clk);valid=0;end endtask
 task check(input[31:0]status,input state);
 begin if(dut.reply[96+:32]!=status || confirmed!=state)$fatal(1,"status/state mismatch %d %d",dut.reply[96+:32],confirmed);end endtask
 integer i;
 initial begin
  repeat(3)@(negedge clk);rstn=1;repeat(3)@(negedge clk);
  if(confirmed || led)$fatal(1,"boot not off");
  send(10,1,7,100);check(0,1);
  if(!led)$fatal(1,"initial phase not on");
  // 250ms half period, no network input needed.
  for(i=0;i<250;i=i+1)begin @(negedge clk);tick=1;now_ms=now_ms+1;@(negedge clk);tick=0;end
  if(led || !confirmed)$fatal(1,"blink or latch failed");
  send(10,1,7,100);check(0,1);if(led)$fatal(1,"duplicate restarted phase");
  send(10,0,7,100);check(5,1);
  send(9,0,7,now_ms);check(2,1);
  send(11,0,6,now_ms);check(1,1);
  send(11,99,7,now_ms);check(4,1);
  now_ms=1000;send(11,0,7,100);check(3,1);
  send(11,2,7,now_ms);check(0,1);if(seq!=10)$fatal(1,"query changed ordering");
  send(11,0,7,now_ms);check(0,0);if(led)$fatal(1,"clear not off");
  send(12,1,7,now_ms);check(0,1);
  @(negedge clk);boot=8;repeat(3)@(negedge clk);
  if(confirmed || led || seq!=0)$fatal(1,"new boot not reset");
  // Sequence wrap uses signed subtraction.
  send(32'hffffffff,1,8,now_ms);check(0,1);
  send(0,0,8,now_ms);check(0,0);
  $display("PASS source indicator: boot, blink, disconnected latch, duplicate/conflict, old seq/boot, TTL, query, clear, wrap");$finish;
 end
 initial begin #100000;$fatal(1,"timeout");end
endmodule
