`timescale 1ns/1ps
module motor_pwm #(parameter integer CLK_HZ=50000000, PWM_HZ=20000)(
 input wire clk,rstn,enable,invert,
 input wire signed [15:0] duty,
 output wire in1,in2
);
 localparam integer PERIOD=CLK_HZ/PWM_HZ;
 // The module's supplied A4950 datasheet allows 200 us to leave standby.
 // Both-high is the specified brake state: wake without applying bridge voltage.
 localparam integer WAKE_CYCLES=(CLK_HZ*64'd250+999999)/1000000;
 localparam integer MIN_DUTY=(1000+PERIOD-1)/PERIOD;
 localparam [1:0] IDLE=0,WAKE=1,RUN=2;
 reg [1:0] state;
 integer wake_left;
 integer phase;
 reg direction;
 reg [15:0] magnitude;
 reg blank;
 wire desired_direction=duty[15]^invert;
 wire [15:0] requested=duty[15] ? -duty : duty;
 wire request_active=requested>=MIN_DUTY;
 wire awake_brake=state==WAKE;
 wire active=state==RUN && !blank && phase<(magnitude*PERIOD/1000);
 assign in1=rstn && enable && request_active && (awake_brake || (active && !direction));
 assign in2=rstn && enable && request_active && (awake_brake || (active && direction));
 always @(posedge clk) begin
   if (!rstn || !enable || !request_active) begin
     state<=IDLE;wake_left<=0;phase<=0;direction<=0;magnitude<=0;blank<=1;
   end else case(state)
     IDLE: begin state<=WAKE;wake_left<=WAKE_CYCLES-1;phase<=0;end
     WAKE: begin
       if(wake_left==0) begin
         state<=RUN;phase<=0;direction<=desired_direction;
         magnitude<=(requested>1000)?1000:requested;blank<=0;
       end else wake_left<=wake_left-1;
     end
     RUN: begin
       if(phase==PERIOD-1) begin
         phase<=0;direction<=desired_direction;
         magnitude<=(requested>1000)?1000:requested;
         blank<=desired_direction!=direction; // one full coast period on reversal
       end else phase<=phase+1;
     end
     default: begin state<=IDLE;wake_left<=0;phase<=0;direction<=0;magnitude<=0;blank<=1;end
   endcase
 end
endmodule
