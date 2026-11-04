`timescale 1ns/1ps
// IEEE754 binary32 radians -> signed milliradians, truncation toward zero.
// Bounded angle input: +/- 8 rad. NaN/Inf/out-of-range rejected.
module float_to_mrad(input wire [31:0] bits,
 output reg signed [31:0] value,output reg valid);
 reg [63:0] magnitude;
 integer shift;
 always @* begin
   value=0;valid=1;magnitude=0;shift=0;
   if (bits[30:23]==255) valid=0;
   else if (bits[30:23]==0) value=0;
   else begin
     magnitude={40'b0,1'b1,bits[22:0]}*64'd1000;
     shift=$signed({1'b0,bits[30:23]})-150;
     if (shift>0) begin
       if (shift>16) begin valid=0;magnitude=0;end
       else magnitude=magnitude<<shift;
     end else if (shift< -63) magnitude=0;
     else magnitude=magnitude>>(-shift);
     if (magnitude>8000) valid=0;
     value=bits[31] ? -$signed(magnitude[31:0]) : $signed(magnitude[31:0]);
   end
 end
endmodule
