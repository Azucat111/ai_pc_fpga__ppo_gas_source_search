`timescale 1ns/1ps
module tb_gas10_integration;
 reg clk=0;always #100 clk=~clk;
 reg rstn=0,wr=0;reg [11:0]addr=0;reg [31:0]data=0;wire [31:0]rd;
 reg serial_in=1;wire serial_out,a,b,c,d;tri1 scl,sda;
 car_core #(.CLK_HZ(5000000)) dut(clk,rstn,wr,addr,12'h0d8,data,4'hf,rd,1'b0,1'b0,1'b0,1'b0,serial_in,a,b,c,d,serial_out,scl,sda,);
 wire [7:0]byte_out;wire valid;wire [31:0]errors;
 uart_rx #(.CLK_HZ(5000000)) rx(clk,rstn,serial_out,byte_out,valid,errors);
 reg [7:0]packet[0:191];integer n=0,main_frames=0,gas_frames=0,i;reg [31:0]last_time=0,stamp;
 reg [15:0]crc;integer j;
 function [15:0]update(input [15:0]x,input [7:0]v);reg [15:0]r;integer k;begin r=x^{v,8'b0};for(k=0;k<8;k=k+1)r=r[15]?(r<<1)^16'h1021:r<<1;update=r;end endfunction
 always @(posedge clk)begin
  if(valid)begin
   packet[n]=byte_out;n=n+1;
   if(n==192)begin
    if(packet[0]!=8'ha5 || packet[1]!=8'h5a)$fatal(1,"UART frames interleaved");
    crc=16'hffff;for(j=0;j<190;j=j+1)crc=update(crc,packet[j]);
    if(crc!={packet[191],packet[190]})$fatal(1,"mixed/CRC frame");
    if(packet[3]==8'h81)begin
     stamp={packet[15],packet[14],packet[13],packet[12]};
     if(main_frames>0 && stamp-last_time!=20)$fatal(1,"GAS scan disturbed50Hz telemetry: %d",stamp-last_time);
     main_frames=main_frames+1;last_time=stamp;
    end else if(packet[3]==8'h82)gas_frames=gas_frames+1;else $fatal(1,"unknown frame type");
    n=0;
   end
  end
  if(dut.tx_busy && dut.gas_busy)$fatal(1,"serializers overlapped");
  if(!dut.drive && (a||b||c||d))$fatal(1,"outputs active after watchdog/disable");
 end
 task write(input[11:0]a0,input[31:0]v);begin @(negedge clk);addr=a0;data=v;wr=1;@(negedge clk);wr=0;end endtask
 task serial_byte(input[7:0]v);integer k;begin serial_in=0;#2170.138889;for(k=0;k<8;k=k+1)begin serial_in=v[k];#2170.138889;end serial_in=1;#2170.138889;end endtask
 reg [7:0]command[0:31];
 initial begin
  $readmemh("build/tests/integration_command.hex",command);
  repeat(4)@(negedge clk);rstn=1;
  write(12'h010,32'h12345678);write(12'h014,1);
  for(i=0;i<32;i=i+1)serial_byte(command[i]);
  for(i=0;i<70;i=i+1)begin write(12'h308,i);write(12'h3b0,1);repeat(10000)@(negedge clk);end
  if(main_frames<6 || gas_frames<5 || errors!=0)$fatal(1,"missing frame types/cadence %d %d",main_frames,gas_frames);
  if(dut.mode!=0 || dut.stop_reason!=2)$fatal(1,"GAS traffic disturbed PL watchdog");
  $display("PASS GAS10 full UART: simultaneous frames, constant50Hz telemetry under commit flood, CRC, PL watchdog and motor gating");$finish;
 end
 initial begin #200000000;$fatal(1,"timeout");end
endmodule
