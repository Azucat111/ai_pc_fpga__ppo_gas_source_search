`timescale 1ns/1ps
module tb_uart_float;
 reg clk=0;always #10 clk=~clk;
 reg rstn=0,valid=0;reg[7:0] data=0;
 wire ready,tx,rxvalid;wire[7:0] rxdata;wire[31:0] errors;
 uart_tx ut(clk,rstn,valid,data,ready,tx);
 uart_rx ur(clk,rstn,tx,rxdata,rxvalid,errors);
 reg[31:0] bits;wire signed[31:0] value;wire good;
 float_to_mrad f(bits,value,good);
 task check(input[31:0] b,input signed[31:0] expected,input valid_expected);
 begin bits=b;#1;if(good!==valid_expected || (good && value!==expected))$fatal(1,"float %h got %d/%d",b,value,good);end endtask
 integer i;
 initial begin
   check(0,0,1);check(32'h80000000,0,1);check(32'h3fc90fdb,1570,1);
   check(32'hbfc90fdb,-1570,1);check(32'h40490fdb,3141,1);
   check(32'h7f800000,0,0);check(32'h7fc00001,0,0);check(32'h41800000,0,0);check(1,0,1);
   repeat(3)@(negedge clk);rstn=1;
   for(i=0;i<8;i=i+1)begin
     wait(ready);@(negedge clk);data=(8'h81<<i)^8'ha5;valid=1;
     @(negedge clk);valid=0;wait(rxvalid);#1;
     if(rxdata!=data || errors)$fatal(1,"UART byte mismatch");
   end
   $display("PASS UART loopback and float decode edge cases");$finish;
 end
 initial begin #1000000;$fatal(1,"test timeout");end
endmodule
