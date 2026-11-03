`timescale 1ns/1ps
module tb_encoder_control;
 reg clk=0;always #5 clk=~clk;
 reg rstn=0,a=0,b=0,invert=0,enable=0,tick=0;
 wire signed [63:0] count;
 wire [31:0] invalid;
 encoder #(.FILTER_CYCLES(3)) e(clk,rstn,a,b,invert,count,invalid);
 reg signed [31:0] target=0,measured=0;
 wire signed [15:0] duty;
 wheel_pi pi(clk,rstn,tick,enable,target,measured,32'd2560,32'd256,32'd65536,16'd250,16'd0,duty);
 reg signed [15:0] pwm_duty=0;
 wire in1,in2;
 motor_pwm #(.CLK_HZ(1000000),.PWM_HZ(10000)) pwm(clk,rstn,enable,1'b0,pwm_duty,in1,in2);
 task phase(input[1:0] s);begin @(negedge clk);{a,b}=s;repeat(12)@(negedge clk);end endtask
 task control_tick;begin @(negedge clk);tick=1;@(negedge clk);tick=0;end endtask
 integer high_count;
 initial begin
   repeat(3)@(negedge clk);rstn=1;repeat(12)@(negedge clk);
   phase(1);phase(3);phase(2);phase(0);
   if(count!=4)$fatal(1,"quadrature forward %d",count);
   phase(2);phase(3);phase(1);phase(0);
   if(count!=0)$fatal(1,"quadrature reverse");
   @(negedge clk);b=1;@(negedge clk);b=0;repeat(12)@(negedge clk);
   if(count!=0)$fatal(1,"glitch counted");
   phase(3);if(invalid!=1 || count!=0)$fatal(1,"illegal transition counted");
   phase(0);invert=1;phase(1);phase(3);phase(2);phase(0);
   if(count!=-4)$fatal(1,"inversion");
   enable=1;target=100*65536;
   repeat(300)control_tick();
   if(duty!=250)$fatal(1,"PI limit %d",duty);
   measured=200*65536;control_tick();
   if(duty>=0)$fatal(1,"PI failed to leave saturation");
   enable=0;repeat(2)@(negedge clk);if(duty!=0 || in1 || in2)$fatal(1,"disable");
   enable=1;pwm_duty=500;repeat(500)@(negedge clk);
   high_count=0;repeat(100)begin @(negedge clk);if(in1)high_count=high_count+1;if(in2)$fatal(1,"wrong PWM direction");end
   if(high_count!=50)$fatal(1,"PWM duty %d",high_count);
   pwm_duty=-500;repeat(300)begin @(negedge clk);if(in1 && in2)$fatal(1,"both inputs high");end
   high_count=0;repeat(100)begin @(negedge clk);if(in2)high_count=high_count+1;if(in1)$fatal(1,"reverse PWM");end
   if(high_count!=50)$fatal(1,"reverse duty");
   rstn=0;#1;if(in1 || in2)$fatal(1,"reset outputs");
   a=1;b=1;repeat(3)@(negedge clk);rstn=1;repeat(20)@(negedge clk);
   if(count!=0 || invalid!=0)$fatal(1,"startup phase must not count as motion");
   $display("PASS encoder/control: quadrature, filter, illegal states, PI saturation/recovery, PWM/reset");$finish;
 end
 initial begin #1000000;$fatal(1,"test timeout");end
endmodule
