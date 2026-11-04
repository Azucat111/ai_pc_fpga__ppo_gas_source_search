`timescale 1ns/1ps
module tb_pwm_diagnostics;
 reg clk=0;always #100 clk=~clk;
 reg rstn=0,wr_en=0,serial_in=1;
 reg [11:0] wr_addr=0,rd_addr=0;
 reg [31:0] wr_data=0;
 wire [31:0] rd_data;
 wire a1,a2,b1,b2,serial_out;
 tri1 scl,sda;
 car_core #(.CLK_HZ(5000000)) dut(clk,rstn,wr_en,wr_addr,rd_addr,wr_data,4'hf,rd_data,
  1'b0,1'b0,1'b0,1'b0,serial_in,a1,a2,b1,b2,serial_out,scl,sda,);
 task write(input [11:0] address,input [31:0] value);begin
  @(negedge clk);wr_addr=address;wr_data=value;wr_en=1;
  @(negedge clk);wr_en=0;
 end endtask
 task expect_read(input [11:0] address,input [31:0] expected);begin
  rd_addr=address;#1;
  if(rd_data!==expected)$fatal(1,"debug register %h got %h expected %h",address,rd_data,expected);
 end endtask
 task serial_byte(input[7:0] data);integer k;begin
  serial_in=0;#2170.138889;
  for(k=0;k<8;k=k+1)begin serial_in=data[k];#2170.138889;end
  serial_in=1;#2170.138889;
 end endtask
 reg [7:0] packet[0:31];
 reg [7:0] reverse_packet[0:31];
 reg [7:0] both_packet[0:31];
 reg [7:0] left_packet[0:31];
 integer i,high_cycles;
 reg [31:0] edge_count;
 initial begin
  $readmemh("build/tests/pwm_diagnostic_command.hex",packet);
  $readmemh("build/tests/pwm_diagnostic_reverse_command.hex",reverse_packet);
  $readmemh("build/tests/pwm_diagnostic_both_command.hex",both_packet);
  $readmemh("build/tests/pwm_diagnostic_left_command.hex",left_packet);
  repeat(4)@(negedge clk);rstn=1;
  expect_read(12'h064,32'h44424731);expect_read(12'h068,5000000);
  write(12'h070,32'hdeadbeef);expect_read(12'h070,0);
  write(12'h010,32'h12345678);
  write(12'h018,3); // Physical M1 is the logical right wheel.
  write(12'h01c,25600);write(12'h020,0);write(12'h024,250);
  write(12'h014,1);
  for(i=0;i<32;i=i+1)serial_byte(packet[i]);
  wait(dut.now_ms>=20);
  expect_read(12'h0a0,1);expect_read(12'h0a4,0);expect_read(12'h0a8,144179);
  expect_read(12'h070,0);expect_read(12'h074,144179);
  expect_read(12'h078,{16'd219,16'd0});
  expect_read(12'h0c0,0);expect_read(12'h0c4,144179);
  @(posedge a1);high_cycles=0;
  for(i=0;i<250;i=i+1)begin
   @(negedge clk);
   if(a1)high_cycles=high_cycles+1;
   if(a2 || b1 || b2)$fatal(1,"Unexpected nonselected motor output");
  end
  if(high_cycles!=54)$fatal(1,"10rpm proportional case: expected 54/250 PWM clocks, got %0d",high_cycles);
  rd_addr=12'h0b0;#1;edge_count=rd_data;
  if(edge_count==0)$fatal(1,"Actual PWM output rising edges were not observed");
  expect_read(12'h0b4,1);expect_read(12'h0b8,0);expect_read(12'h0bc,0); // Selected bridge wake/brake edge.
  wait(dut.now_ms>=120);repeat(4)@(negedge clk);
  expect_read(12'h078,0);expect_read(12'h07c,0);
  expect_read(12'h0ac,{16'd219,16'd0});expect_read(12'h0c4,144179);
  rd_addr=12'h0b0;#1;
  if(rd_data<=edge_count)$fatal(1,"PWM edge counter failed to retain activity");
  write(12'h0a8,0);expect_read(12'h0a8,144179);
  if(dut.mode!=0 || dut.stop_reason!=2)$fatal(1,"Watchdog STOP changed");
  // Static no-load driver test: original UART -> motion -> PI -> PWM chain,
  // with saturated 100% duty. No firmware or RTL override of output pins.
  rstn=0;repeat(4)@(negedge clk);rstn=1;
  write(12'h010,32'h12345678);write(12'h018,3);
  write(12'h01c,256000);write(12'h020,0);write(12'h024,1000);
  write(12'h014,1);
  for(i=0;i<32;i=i+1)serial_byte(packet[i]);
  wait(dut.now_ms>=20);
  expect_read(12'h078,{16'd1000,16'd0});
  repeat(500)begin @(negedge clk);
    if(!a1 || a2 || b1 || b2)$fatal(1,"No-load positive static output");
  end
  for(i=0;i<32;i=i+1)serial_byte(reverse_packet[i]);
  // The target changes on command receipt, but PI and duty update at 100 Hz.
  // Allow the 30 ms tick and the following PWM boundary to complete.
  wait(dut.now_ms>=31);
  expect_read(12'h078,{-16'sd1000,16'd0});
  repeat(500)begin @(negedge clk);
    if(a1 || !a2 || b1 || b2)$fatal(1,"No-load reverse static output");
  end
  for(i=0;i<32;i=i+1)serial_byte(both_packet[i]);
  wait(dut.now_ms>=51);
  expect_read(12'h078,{16'd1000,16'd1000});
  repeat(500)begin @(negedge clk);
    if(!a1 || a2 || !b1 || b2)$fatal(1,"Both bridge no-load static output");
  end
  for(i=0;i<32;i=i+1)serial_byte(left_packet[i]);
  wait(dut.now_ms>=61);
  expect_read(12'h078,{16'd0,16'd1000});
  repeat(500)begin @(negedge clk);
    if(a1 || a2 || !b1 || b2)$fatal(1,"B-only no-load static output");
  end
  write(12'h014,0);repeat(4)@(negedge clk);
  expect_read(12'h078,0);expect_read(12'h07c,0);
  rstn=0;repeat(4)@(negedge clk);rstn=1;
  write(12'h010,32'h12345678);write(12'h018,3);
  write(12'h01c,25600);write(12'h020,0);write(12'h024,150);
  write(12'h014,1);
  for(i=0;i<32;i=i+1)serial_byte(packet[i]);
  wait(dut.now_ms>=20);expect_read(12'h078,{16'd150,16'd0});
  @(posedge a1);high_cycles=0;
  for(i=0;i<250;i=i+1)begin @(negedge clk);
    if(a1)high_cycles=high_cycles+1;
    if(a2 || b1 || b2)$fatal(1,"Unexpected output during single motor pulse");
  end
  if(high_cycles!=37)$fatal(1,"Motor pulse cap expected 37/250 clocks, got %0d",high_cycles);
  write(12'h014,0);repeat(4)@(negedge clk);expect_read(12'h07c,0);
  // Explicit 70% diagnostic: original command path saturates at 700 permille.
  rstn=0;repeat(4)@(negedge clk);rstn=1;
  write(12'h010,32'h12345678);write(12'h018,3);
  write(12'h01c,256000);write(12'h020,0);write(12'h024,700);
  write(12'h014,1);
  for(i=0;i<32;i=i+1)serial_byte(packet[i]);
  wait(dut.now_ms>=20);expect_read(12'h078,{16'd700,16'd0});
  @(posedge a1);high_cycles=0;
  for(i=0;i<250;i=i+1)begin @(negedge clk);
    if(a1)high_cycles=high_cycles+1;
    if(a2 || b1 || b2)$fatal(1,"Unexpected output during 70 percent pulse");
  end
  if(high_cycles!=175)$fatal(1,"70 percent expected 175/250 clocks, got %0d",high_cycles);
  write(12'h014,0);repeat(4)@(negedge clk);expect_read(12'h07c,0);
  $display("PASS motor diagnostics: 54/250 PWM, retained diagnostics, watchdog STOP; no-load +/-100 percent/both/B-only; motor pulse 37/250 and 175/250 caps and disable");
  $finish;
 end
 initial begin #250000000;$fatal(1,"PWM diagnostic test timeout");end
endmodule
