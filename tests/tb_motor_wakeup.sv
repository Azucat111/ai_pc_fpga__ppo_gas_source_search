`timescale 1ns/1ps
module tb_motor_wakeup;
 reg clk=0;always #100 clk=~clk;
 reg rstn=0,enable=0,invert=0,allow_brake=0;
 reg signed [15:0] duty=0;
 wire in1,in2;
 motor_pwm #(.CLK_HZ(5000000)) dut(clk,rstn,enable,invert,duty,in1,in2);
 always @(posedge clk) begin
   if(rstn && !allow_brake && in1 && in2)$fatal(1,"unexpected brake outside wake");
   if(!enable && (in1 || in2))$fatal(1,"output while disabled");
 end
 task wake(input signed [15:0] value,input integer reverse);
   time began;
   begin
     @(negedge clk);allow_brake=1;duty=value;enable=1;
     wait(in1 && in2);began=$time;
     wait(!(in1 && in2));
     if($time-began!=250000)$fatal(1,"wake must last 250 us, got %0t",$time-began);
     #1;
     if(reverse ? (!in2 || in1) : (!in1 || in2))$fatal(1,"wrong first output after wake");
     allow_brake=0;
   end
 endtask
 task sample_period(input integer reverse,input integer expected);
   integer i,n;
   begin
     if(reverse)@(posedge in2);else @(posedge in1);
     n=0;
     for(i=0;i<250;i=i+1)begin @(negedge clk);
       if(reverse ? in1 : in2)$fatal(1,"wrong direction during PWM");
       if(reverse ? in2 : in1)n=n+1;
     end
     if(n!=expected)$fatal(1,"PWM expected %0d/250, got %0d",expected,n);
   end
 endtask
 integer i;
 initial begin
   repeat(4)@(negedge clk);rstn=1;enable=1;
   repeat(6000)begin @(negedge clk);if(in1 || in2)$fatal(1,"zero duty must not wake");end
   wake(150,0);sample_period(0,37);
   // A nonzero reversal retains the existing blank period, without re-braking.
   @(negedge clk);duty=-500;
   wait(in2);sample_period(1,125);
   @(negedge clk);duty=0;#1;if(in1 || in2)$fatal(1,"zero did not coast immediately");
   repeat(10000)@(negedge clk);
   invert=1;wake(500,1);sample_period(1,125);
   @(negedge clk);enable=0;#1;if(in1 || in2)$fatal(1,"disable did not coast");
   // STOP cancels a wake in progress and next start must get a complete wake.
   @(negedge clk);allow_brake=1;enable=1;
   wait(in1 && in2);repeat(500)@(negedge clk);
   enable=0;#1;if(in1 || in2)$fatal(1,"STOP during wake");
   repeat(4)@(negedge clk);enable=1;
   wait(in1 && in2);repeat(500)@(negedge clk);
   rstn=0;#1;if(in1 || in2)$fatal(1,"reset during wake");
   @(negedge clk);enable=0;invert=0;duty=0;
   repeat(4)@(negedge clk);rstn=1;
   wake(1000,0);
   repeat(500)begin @(negedge clk);if(!in1 || in2)$fatal(1,"100 percent output");end
   @(negedge clk);duty=3;
   repeat(10000)begin @(negedge clk);if(in1 || in2)$fatal(1,"quantized zero duty must not wake");end
   allow_brake=0;enable=0;
   $display("PASS A4950 wake: exact 250us brake, zero/quantized-zero idle, 20kHz duty, reverse/invert, standby restart, STOP/reset cancellation");
   $finish;
 end
 initial begin #10000000;$fatal(1,"wake test timeout");end
endmodule
