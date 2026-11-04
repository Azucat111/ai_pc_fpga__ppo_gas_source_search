`timescale 1ns/1ps
module tb_tgs_features;
 reg clk=0;always #5 clk=~clk;
 reg rstn=0;reg [31:0] now_ms=0,t0=0,t1=0;
 reg signed [15:0] x0=0,x1=0;reg [1:0] valid=0;
 reg wr=0;reg [11:0] wa=0,ra=0;reg [31:0] wd=0;wire [31:0] rd;
 tgs_features dut(clk,rstn,now_ms,x0,x1,valid,t0,t1,wr,wa,ra,wd,4'hf,rd);
 task sample(input integer x,y);
 begin @(negedge clk);now_ms=now_ms+100;t0=now_ms;t1=now_ms;x0=x;x1=y;valid=3;
 repeat(4)@(negedge clk);end endtask
 task write(input [11:0] a);begin @(negedge clk);wa=a;wd=1;wr=1;@(negedge clk);wr=0;end endtask
 integer i;
 initial begin
 repeat(3)@(negedge clk);rstn=1;
 sample(1000,-1000);
 if(dut.c0!=1 || dut.c1!=1 || dut.used!=1)$fatal(1,"first simultaneous samples lost");
 repeat(20)@(negedge clk);
 if(dut.c0!=1)$fatal(1,"duplicate timestamp reprocessed");
 for(i=0;i<12;i=i+1)sample(3000,-3000);
 if(!dut.a0 || !dut.a1 || dut.e0!=1 || dut.e1!=1)$fatal(1,"symmetric threshold event missing");
 if(dut.d0<=0 || dut.d1>=0)$fatal(1,"signed math failed");
 ra=12'h550;#1;if($signed(rd)!=1000)$fatal(1,"FIFO head changed");
 for(i=0;i<10;i=i+1)sample(3000,-3000);
 if(dut.used!=16 || dut.overflow==0)$fatal(1,"overflow not counted");
 write(12'h520);if(dut.used!=15)$fatal(1,"pop failed");
 write(12'h524);if(dut.n0!=0 || dut.peak0!=0)$fatal(1,"window reset failed");
 // Invalid data must not produce samples, even if its timestamp changes.
 valid=0;t0=t0+1;repeat(4)@(negedge clk);
 if(dut.n0!=0)$fatal(1,"invalid sample accepted");
 valid=3;
 for(i=0;i<60;i=i+1)sample(1000,-1000);
 if(dut.a0 || dut.a1)$fatal(1,"hysteresis did not clear");
 // Reset then change live valid immediately after conversion capture.
 @(negedge clk);rstn=0;valid=0;repeat(2)@(negedge clk);rstn=1;
 @(negedge clk);valid=3;t0=123;t1=124;x0=77;x1=-88;
 @(negedge clk);valid=0;x0=999;x1=999;
 repeat(2)@(negedge clk);ra=12'h548;#1;
 if(rd[7:6]!=3 || rd[3:2]!=3)$fatal(1,"sample epoch validity mismatch");
 ra=12'h550;#1;if($signed(rd)!=77)$fatal(1,"sample raw epoch mismatch");
 ra=12'h554;#1;if($signed(rd)!=-88)$fatal(1,"sample raw1 epoch mismatch");
 $display("PASS TGS: dual lanes, signed math, event/deassert, duplicate/invalid rejection, FIFO stable/full/pop, window");$finish;
 end
 initial begin #1000000;$fatal(1,"timeout");end
endmodule
