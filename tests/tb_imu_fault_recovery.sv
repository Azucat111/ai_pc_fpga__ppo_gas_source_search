`timescale 1ns/1ps
// A failed transfer must retain the last good sample; motion still expires at 150ms.
module tb_imu_fault_recovery;
 reg clk=0; always #500 clk=~clk;
 reg rstn=0; reg [31:0] now_ms=0; integer divider=0;
 always @(posedge clk) begin
   if(!rstn) begin divider<=0;now_ms<=0;end
   else if(divider==999) begin divider<=0;now_ms<=now_ms+1;end
   else divider<=divider+1;
 end
 tri1 scl,sda;
 i2c_slave_model #(.ADDRESS(7'h23)) imu(scl,sda);
 i2c_slave_model #(.ADDRESS(7'h48),.ADS(1)) adc(scl,sda);
 wire ps_busy,ps_done,ps_error,imu_valid,imu_new;
 wire [255:0] ps_data;
 wire signed [31:0] yaw,gyro;
 wire [31:0] imu_time,imu_errors,adc_errors,t0,t1;
 wire signed [15:0] a0,a1; wire [1:0] adc_valid;
 sensors #(.CLK_HZ(1000000)) dut(clk,rstn,now_ms,1'b0,7'h77,8'hd0,6'd0,6'd0,256'd0,
   ps_busy,ps_done,ps_error,ps_data,imu_valid,imu_new,yaw,gyro,imu_time,imu_errors,adc_errors,
   a0,a1,adc_valid,t0,t1,scl,sda);
 reg cmd_valid=0;reg [255:0] cmd=0;
 wire [31:0] mode,action_id,action_start,last_seq,stop_reason,reject_seq,status;
 wire signed [31:0] target_l,target_r;
 motion control(clk,rstn,1'b0,1'b1,1'b1,now_ms,32'h12345678,cmd_valid,cmd,
   imu_valid,imu_new,imu_time,32'd150,yaw,gyro,32'd1310720,32'd512,32'd128,32'd600,
   32'd1000,32'd1000,32'd20,32'd80,mode,action_id,action_start,last_seq,stop_reason,
   reject_seq,status,target_l,target_r);
 integer saved_time,saved_errors,phase_time;
 reg signed [31:0] saved_yaw,saved_gyro;
 task straight(input [31:0] seq);
   begin
     @(negedge clk);cmd=0;cmd[63:32]=seq;cmd[95:64]=32'h12345678;
     cmd[127:96]=now_ms;cmd[135:128]=2;cmd[159:144]=500;
     cmd[191:160]=65536;cmd[223:192]=65536;cmd_valid=1;
     @(negedge clk);cmd_valid=0;
   end
 endtask
 initial begin
   #1;imu.memory[14]=0;imu.memory[15]=16;
   imu.memory[46]=8'hdb;imu.memory[47]=8'h0f;imu.memory[48]=8'hc9;imu.memory[49]=8'h3f;
   force imu.ack=0;
   #3000;@(negedge clk);rstn=1;
   wait(now_ms>=55);#1;
   if(imu_valid || imu_time!=0)$fatal(1,"No valid sample may be invented before first success");
   straight(1);#1;if(status!=7 || mode!=0)$fatal(1,"Heading must reject never-valid IMU");
   release imu.ack;
   wait(imu_valid && imu_time>55);wait(!dut.bus.busy);@(negedge clk);
   force imu.ack=0;
   saved_time=imu_time;saved_errors=imu_errors;saved_yaw=yaw;saved_gyro=gyro;
   straight(2);#1;if(mode!=2)$fatal(1,"Heading start on fresh IMU failed");
   wait(now_ms>=saved_time+60);#1;
   if(imu_errors<=saved_errors)$fatal(1,"Expected injected I2C failures");
   if(!imu_valid || imu_time!=saved_time || yaw!=saved_yaw || gyro!=saved_gyro)
     $fatal(1,"Failed transfer must preserve sample, validity and original timestamp");
   if(mode!=2)$fatal(1,"Fresh last-good sample must tolerate a transient transfer failure");
   wait(now_ms>=saved_time+151);repeat(3)@(negedge clk);
   if(mode!=0 || stop_reason!=3)$fatal(1,"150ms stale IMU must stop heading");
   straight(3);#1;if(status!=7 || mode!=0)$fatal(1,"Stale sample must reject a new heading command");
   release imu.ack;
   wait(imu_time>saved_time);straight(4);#1;
   if(mode!=2)$fatal(1,"Fresh recovery must allow a new explicit command");
   // A decoded NaN is also an invalid update, never a replacement sample.
   wait(!dut.bus.busy);@(negedge clk);
   imu.memory[46]=0;imu.memory[47]=0;imu.memory[48]=8'hc0;imu.memory[49]=8'h7f;
   saved_time=imu_time;saved_yaw=yaw;saved_errors=imu_errors;phase_time=now_ms;
   wait(now_ms>=phase_time+50);#1;
   if(imu_errors<=saved_errors || !imu_valid || imu_time!=saved_time || yaw!=saved_yaw || mode!=2)
     $fatal(1,"Invalid float must retain fresh last-good sample without refreshing its timestamp");
   @(negedge clk);rstn=0;repeat(3)@(negedge clk);
   if(imu_valid || imu_time!=0)$fatal(1,"Reset must clear last-good validity");
   $display("PASS IMU last-good: first validity, NACK/NaN retention, 150ms stop, stale rejection and recovery");
   $finish;
 end
 initial begin #500000000;$fatal(1,"test timeout");end
endmodule
