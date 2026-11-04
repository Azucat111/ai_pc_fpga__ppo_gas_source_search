`timescale 1ns/1ps
module tb_encoder_wiring;
 reg clk=0;always #5 clk=~clk;
 reg rstn=0,actual_a=0,actual_b=0;
 wire signed [63:0] reference_count,swapped_raw_count,corrected_count;
 wire [31:0] reference_invalid,swapped_raw_invalid,corrected_invalid;
 encoder #(.FILTER_CYCLES(3)) reference_encoder(
   clk,rstn,actual_a,actual_b,1'b0,reference_count,reference_invalid);
 // PCB A receives physical B, and PCB B receives physical A on both motors.
 encoder #(.FILTER_CYCLES(3)) swapped_raw_encoder(
   clk,rstn,actual_b,actual_a,1'b0,swapped_raw_count,swapped_raw_invalid);
 encoder #(.FILTER_CYCLES(3)) corrected_encoder(
   clk,rstn,actual_b,actual_a,1'b1,corrected_count,corrected_invalid);
 task phase(input [1:0] state);
   begin @(negedge clk);{actual_a,actual_b}=state;repeat(12)@(negedge clk);end
 endtask
 task check_count(input integer expected);
   begin
     if(reference_count!=expected || corrected_count!=expected || swapped_raw_count!=-expected)
       $fatal(1,"AB correction expected %d, reference %d, corrected %d, raw %d",
              expected,reference_count,corrected_count,swapped_raw_count);
   end
 endtask
 integer i;
 initial begin
   repeat(3)@(negedge clk);rstn=1;repeat(12)@(negedge clk);
   // 330 physical periods = 1320 x4 counts per wheel revolution.
   for(i=0;i<330;i=i+1)begin phase(1);phase(3);phase(2);phase(0);end
   check_count(1320);
   for(i=0;i<330;i=i+1)begin phase(2);phase(3);phase(1);phase(0);end
   check_count(0);
   phase(2);phase(3);phase(1);phase(0);check_count(-4);
   // Swapping the wires does not conceal invalid simultaneous transitions.
   phase(3);check_count(-4);
   if(reference_invalid!=1 || swapped_raw_invalid!=1 || corrected_invalid!=1)
     $fatal(1,"AB swap changed illegal-transition detection");
   $display("PASS encoder wiring: swapped AB correction, 1320 counts/rev, reverse and illegal transitions");
   $finish;
 end
 initial begin #1000000;$fatal(1,"test timeout");end
endmodule
