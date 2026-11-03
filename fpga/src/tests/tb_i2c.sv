`timescale 1ns/1ps
module tb_i2c;
 reg clk=0;always #500 clk=~clk;
 reg rstn=0,start=0,stretch=0;
 tri1 scl,sda;
 assign scl=stretch?1'b0:1'bz;
 reg [6:0] address=7'h23;reg[7:0] register_address=0;
 reg[5:0] wlen=0,rlen=0;reg[255:0] wdata=0;
 wire busy,done,error;wire[255:0] rdata;
 i2c_master #(.CLK_HZ(1000000)) dut(clk,rstn,start,address,register_address,wlen,rlen,wdata,busy,done,error,rdata,scl,sda);
 i2c_slave_model slave(scl,sda);
 task launch(input[7:0] rega,input[5:0] nwrite,nread,input[255:0] data);
 begin @(negedge clk);register_address=rega;wlen=nwrite;rlen=nread;wdata=data;start=1;
 @(negedge clk);start=0;end endtask
 task finish_ok;begin wait(done);#1;if(error)$fatal(1,"unexpected I2C error");@(negedge clk);end endtask
 initial begin
   #3000;@(negedge clk);rstn=1;
   launch(8'h30,3,0,24'h563412);finish_ok();
   if(slave.memory[48]!=8'h12 || slave.memory[49]!=8'h34 || slave.memory[50]!=8'h56)$fatal(1,"I2C write mismatch");
   launch(8'h30,0,3,0);
   wait(dut.state==3);@(negedge clk);stretch=1;#100000;stretch=0;
   finish_ok();if(rdata[23:0]!=24'h563412)$fatal(1,"I2C repeated START/read %h",rdata[23:0]);
   address=7'h24;launch(0,0,1,0);wait(done);#1;
   if(!error)$fatal(1,"missing NACK detection");@(negedge clk);
   stretch=1;address=7'h23;launch(0,0,1,0);wait(done);#1;
   if(!error || busy)$fatal(1,"missing stuck bus timeout");stretch=0;
   @(negedge clk);launch(8'h30,0,1,0);finish_ok();
   if(rdata[7:0]!=8'h12)$fatal(1,"bus did not recover");
   $display("PASS I2C: multi-byte write/read, repeated START, stretching, NACK, timeout/recovery");$finish;
 end
 initial begin #60000000;$fatal(1,"test timeout");end
endmodule
