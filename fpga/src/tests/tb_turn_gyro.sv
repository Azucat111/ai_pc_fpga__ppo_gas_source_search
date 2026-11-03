`timescale 1ns/1ps
module tb_turn_gyro;
 reg clk=0;always #5 clk=~clk;
 reg rstn=0,tick=0,valid=0,imu_new=0,imu_valid=1,enabled=1;
 reg [31:0] now_ms=0,imu_time=0,seq=0;
 reg signed [31:0] yaw=0,gyro=0;
 reg [255:0] cmd=0;
 wire [31:0] mode,action,started,last_seq,reason,reject,status;
 wire signed [31:0] left,right;
 motion dut(clk,rstn,tick,enabled,1'b1,now_ms,32'h12345678,valid,cmd,
 imu_valid,imu_new,imu_time,32'd150,yaw,gyro,32'd2621440,32'd512,32'd128,32'd600,
 32'd655360,32'd655360,32'd20,32'd80,mode,action,started,last_seq,reason,reject,status,left,right);
 task send(input [7:0] requested,input signed [31:0] value);
  begin @(negedge clk);seq=seq+1;cmd=0;cmd[63:32]=seq;cmd[95:64]=32'h12345678;
   cmd[127:96]=now_ms;cmd[135:128]=requested;cmd[159:144]=500;
   cmd[191:160]=value;cmd[223:192]=(requested==2)?value:0;valid=1;
   @(negedge clk);valid=0;#1;
  end
 endtask
 task control_step;
  begin @(negedge clk);now_ms=now_ms+10;imu_time=now_ms;tick=1;imu_new=1;
   @(negedge clk);tick=0;imu_new=0;#1;
  end
 endtask
 integer i;
 task steps(input integer count);
  begin for(i=0;i<count;i=i+1)begin control_step();if(i%10==0)send(4,action);end end
 endtask
 initial begin
  #20;@(negedge clk);rstn=1;
  // Reproduce the observed early-finish problem: fused yaw says90 but gyro is83.
  send(3,1571);gyro=500;yaw=1571;steps(290);gyro=0;steps(22);
  if(mode!=3 || dut.error<100 || left>=0 || right<=0)
    $fatal(1,"fused heading prematurely completed the turn");
  if(dut.straight_gyro_sum!=145000)$fatal(1,"keepalive reset turn integral");
  // Finish by body gyro, even though fusion jumps/wraps in the wrong direction.
  yaw=-2500;gyro=500;steps(24);gyro=0;steps(22);
  if(mode!=0 || reason!=4)$fatal(1,"positive gyro turn failed to complete");
  send(3,-1571);
  if(dut.straight_gyro_sum!=0 || left<=0 || right>=0)$fatal(1,"negative action did not reset");
  gyro=-500;steps(314);gyro=100;steps(19);
  if(mode!=3)$fatal(1,"completed despite angular rate above tolerance");
  gyro=0;steps(19);
  if(mode!=3)$fatal(1,"completion did not require200ms settled");
  steps(2);if(mode!=0 || reason!=4)$fatal(1,"negative turn did not settle");
  // Signed fractional contributions must accumulate, not truncate each tick.
  send(3,-50);gyro=-1;steps(100);gyro=0;
  if(dut.straight_gyro_sum!=-100 || dut.error!=-49)$fatal(1,"fractional negative gyro lost");
  // Fresh action replaces any partial angle; duplicate action cannot replay it.
  send(3,4712);gyro=1000;steps(100);gyro=0;
  if(dut.error!=3712)$fatal(1,"large turn wrapped or retained previous angle");
  @(negedge clk);valid=1;@(negedge clk);valid=0;#1;
  if(dut.straight_gyro_sum!=100000)$fatal(1,"duplicate reset integral");
  gyro=1000;steps(371);gyro=0;steps(22);
  if(mode!=0 || reason!=4)$fatal(1,"270degree unwrapped turn failed");
  send(3,1571);gyro=500;steps(5);imu_valid=0;repeat(3)@(negedge clk);
  if(mode!=0 || reason!=3 || left!=0 || right!=0)$fatal(1,"IMU loss did not stop turn");
  imu_valid=1;imu_time=now_ms;send(3,1571);enabled=0;repeat(3)@(negedge clk);
  if(mode!=0 || reason!=1 || left!=0 || right!=0)$fatal(1,"disable did not stop turn");
  enabled=1;send(3,1571);now_ms=now_ms+501;imu_time=now_ms;repeat(3)@(negedge clk);
  if(mode!=0 || reason!=2)$fatal(1,"turn lease did not expire");
  $display("PASS gyro relative turn: premature fused completion, signs, settle/rate, fractions,270deg, reset, duplicate, IMU/disable/lease");$finish;
 end
 initial begin #1000000;$fatal(1,"test timeout");end
endmodule
