`timescale 1ns/1ps
// A slave left transmitting a zero after an interrupted read holds SDA low.
// Releasing the master's pins alone cannot recover this I2C state.
module tb_i2c_bus_clear;
 reg clk=0;always #500 clk=~clk;
 reg rstn=0,start=0,stretch=0;
 tri1 scl,sda;
 assign scl=stretch?1'b0:1'bz;
 wire busy,done,error;wire [255:0] data;
 i2c_master #(.CLK_HZ(1000000)) dut(clk,rstn,start,7'h23,8'h30,6'd0,6'd1,256'd0,busy,done,error,data,scl,sda);
 i2c_slave_model slave(scl,sda);
 task launch;begin @(negedge clk);start=1;@(negedge clk);start=0;end endtask
 integer pulses=0;
 always @(posedge scl)if(rstn && dut.error)pulses=pulses+1;
 initial begin
   #1;slave.memory[48]=0;
   #3000;@(negedge clk);rstn=1;
   launch();wait(dut.stage==4 && dut.state==3 && dut.bit_index==5);
   @(negedge clk);stretch=1;
   // Force a timeout, then release SCL while the slave is still sending zero.
   #20500000;stretch=0;
   wait(!busy);@(negedge clk);
   if(!error)$fatal(1,"interrupted read must fail, not invent data");
   @(negedge clk);launch();wait(done);#1;
   // A bus-clear transaction may itself report failure, then the retry succeeds.
   if(error)begin @(negedge clk);launch();wait(done);#1;end
   if(error || data[7:0]!=0 || sda!==1'b1)$fatal(1,"slave SDA held low after timeout: bus clear missing");
   if(pulses<9)$fatal(1,"must generate nine recovery clocks");
   $display("PASS I2C interrupted zero-byte read: nine clocks + STOP, failed sample rejected, retry succeeds");$finish;
 end
 initial begin #80000000;$fatal(1,"bus clear test timeout");end
endmodule
