`timescale 1ns/1ps
module tb_sensors_fault_fairness;
 reg clk=0;always #500 clk=~clk;
 reg rstn=0,request=0;reg [31:0] now_ms=0;
 integer div=0;
 always @(posedge clk)if(rstn)begin if(div==999)begin div<=0;now_ms<=now_ms+1;end else div<=div+1;end
 // Permanent SCL fault makes each IMU transaction exceed its 20ms period.
 tri1 scl,sda;assign scl=1'b0;
 wire ps_busy,ps_done,ps_error;wire [255:0] ps_data;
 wire valid,new_sample;wire signed [31:0] yaw,gyro;
 wire [31:0] imu_time,imu_errors,adc_errors,t0,t1;
 wire signed [15:0] a0,a1;wire [1:0] adc_valid;
 sensors #(.CLK_HZ(1000000)) dut(clk,rstn,now_ms,request,7'h77,8'hd0,6'd0,6'd1,256'd0,
 ps_busy,ps_done,ps_error,ps_data,valid,new_sample,yaw,gyro,imu_time,imu_errors,adc_errors,a0,a1,adc_valid,t0,t1,scl,sda);
 initial begin
   #3000;@(negedge clk);rstn=1;
   wait(now_ms==25);@(negedge clk);request=1;@(negedge clk);request=0;
   wait(ps_done);#1;
   if(!ps_error || valid)$fatal(1,"fault must remain invalid");
   wait(now_ms==400);#1;
   if(!imu_errors || !adc_errors)$fatal(1,"one failed device starved another bus client");
   $display("PASS sensors: persistent fault cannot starve PS mailbox/ADC; all failures stay invalid");$finish;
 end
 initial begin #500000000;$fatal(1,"PS mailbox starved by overdue IMU retries");end
endmodule
