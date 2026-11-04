`timescale 1ns/1ps
// Q16 counts per 10 ms. kp/ki: Q8 permille per count (ki includes dt).
module wheel_pi(
 input wire clk,rstn,tick,enable,
 input wire signed [31:0] target,measured,
 input wire [31:0] kp,ki,ramp,
 input wire [15:0] duty_limit,deadzone,
 output reg signed [15:0] duty
);
 reg signed [31:0] ramped;
 reg signed [63:0] integral;
 reg signed [63:0] next_target,error,p_term,next_i,total,limit64,out64;
 always @* begin
   next_target=$signed(target);
   if ($signed(target)>$signed(ramped)+$signed({1'b0,ramp}))
     next_target=$signed(ramped)+$signed({1'b0,ramp});
   if ($signed(target)<$signed(ramped)-$signed({1'b0,ramp}))
     next_target=$signed(ramped)-$signed({1'b0,ramp});
   error=next_target-$signed(measured);
   p_term=(error*$signed({1'b0,kp}))>>>24;
   next_i=integral+error*$signed({1'b0,ki});
   limit64=(duty_limit>1000)?1000:duty_limit;
   total=p_term+(next_i>>>24);
   // Conditional integration prevents winding farther into saturation.
   if ((total>limit64 && error>0)||(total< -limit64 && error<0))
     next_i=integral;
   if (next_i>(limit64<<<24)) next_i=limit64<<<24;
   if (next_i< -(limit64<<<24)) next_i= -(limit64<<<24);
   out64=p_term+(next_i>>>24);
   if (out64>0 && out64<deadzone) out64=deadzone;
   if (out64<0 && out64> -$signed({1'b0,deadzone})) out64= -$signed({1'b0,deadzone});
   if (out64>limit64) out64=limit64;
   if (out64< -limit64) out64= -limit64;
 end
 always @(posedge clk) begin
   if (!rstn || !enable) begin duty<=0;integral<=0;ramped<=0;end
   else if (tick) begin
     ramped<=next_target[31:0];integral<=next_i;duty<=out64[15:0];
   end
 end
endmodule
