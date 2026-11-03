`timescale 1ns/1ps
module tb_board_clock;
 reg clk=0;always #100 clk=~clk; // Real deployed 5 MHz fabric clock.
 reg rstn=0,rx=1,tx_valid=0,enable=0;
 reg [7:0] tx_data=0;
 wire [7:0] rx_data;wire rx_valid,tx_ready,tx;
 wire [31:0] errors;
 uart_rx #(.CLK_HZ(5000000)) receiver(clk,rstn,rx,rx_data,rx_valid,errors);
 uart_tx #(.CLK_HZ(5000000)) sender(clk,rstn,tx_valid,tx_data,tx_ready,tx);
 reg signed [15:0] duty=500;
 wire in1,in2;
 motor_pwm #(.CLK_HZ(5000000)) pwm(clk,rstn,enable,1'b0,duty,in1,in2);
 integer rx_count=0,tx_count=0,k,b,high_count;
 reg [7:0] expected[0:767];reg [7:0] received;
 real bit_ns=2170.138889;
 always @(posedge clk) if(rx_valid)begin
   if(rx_data!==expected[rx_count])$fatal(1,"external UART RX byte %d got %h expected %h",rx_count,rx_data,expected[rx_count]);
   rx_count=rx_count+1;
 end
 // Independent ideal-rate receiver, not a loopback sharing the rounded divider.
 initial begin
   wait(rstn);
   for(tx_count=0;tx_count<256;tx_count=tx_count+1)begin
     @(negedge tx);#(2170.138889*1.5);
     for(b=0;b<8;b=b+1)begin received[b]=tx;#2170.138889;end
     if(tx!==1 || received!==tx_count[7:0])$fatal(1,"external UART TX byte %d got %h",tx_count,received);
   end
 end
 task send_external(input [7:0] value);
 integer n;begin
   rx=0;#(bit_ns);
   for(n=0;n<8;n=n+1)begin rx=value[n];#(bit_ns);end
   rx=1;#(bit_ns);
 end endtask
 initial begin
   for(k=0;k<768;k=k+1)expected[k]=k;
   repeat(5)@(negedge clk);rstn=1;#37;
   // Back-to-back frames with nominal and +/-1% external clock error.
   for(k=0;k<768;k=k+1)begin
     if(k==256)bit_ns=2170.138889/1.01;
     if(k==512)bit_ns=2170.138889/0.99;
     send_external(k[7:0]);
   end
   repeat(30)@(negedge clk);
   if(rx_count!=768 || errors)$fatal(1,"UART receive count/errors %d %d",rx_count,errors);
   for(k=0;k<256;k=k+1)begin
     wait(tx_ready);@(negedge clk);tx_data=k;tx_valid=1;
     @(negedge clk);tx_valid=0;wait(!tx_ready);wait(tx_ready);
   end
   wait(tx_count==256);
   enable=1;repeat(2000)@(negedge clk);
   high_count=0;
   repeat(250)begin @(negedge clk);if(in1)high_count=high_count+1;if(in2)$fatal(1,"PWM polarity");end
   if(high_count!=125)$fatal(1,"20 kHz / 50 percent PWM at 5 MHz: %d",high_count);
   enable=0;@(negedge clk);if(in1 || in2)$fatal(1,"PWM disable");
   $display("PASS board clock: 5 MHz, external 460800 UART +/-1 percent, 20 kHz PWM");$finish;
 end
 initial begin #30000000;$fatal(1,"test timeout");end
endmodule
