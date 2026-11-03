`timescale 1ns/1ps
module tb_straight_gyro;
 reg clk=0;always #5 clk=~clk;
 reg rstn=0,tick=0,valid=0,imu_new=0,imu_valid=1;
 reg [31:0] now_ms=0,imu_time=0,seq=0;
 reg signed [31:0] yaw=0,gyro=0;
 reg [255:0] cmd=0;
 wire [31:0] mode,action,started,last_seq,reason,reject,status;
 wire signed [31:0] left,right;
 motion dut(clk,rstn,tick,1'b1,1'b1,now_ms,32'h12345678,valid,cmd,
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
 reg signed [31:0] remembered_left,remembered_right;
 initial begin
  #20;@(negedge clk);rstn=1;
  send(2,655360);gyro=-100;yaw=200;
  for(i=0;i<100;i=i+1)begin control_step();if(i%10==0)send(4,action);end
  gyro=0;#1;
  if(mode!=2 || !(left<655360 && right>655360))$fatal(1,"wrong-sign magnetic yaw drove wrong correction");
  if(dut.straight_gyro_sum!=-10000)$fatal(1,"gyro integration/heartbeat reset");
  remembered_left=left;remembered_right=right;yaw=-2000;#1;
  if(left!=remembered_left || right!=remembered_right)$fatal(1,"magnetic yaw changed straight target");
  send(0,0);send(2,655360);#1;
  if(left!=655360 || right!=655360 || dut.straight_gyro_sum!=0)$fatal(1,"new straight action retained drift");
  gyro=-1;
  for(i=0;i<100;i=i+1)begin control_step();if(i%10==0)send(4,action);end
  gyro=0;#1;
  if(dut.straight_gyro_sum!=-100 || left>=655360 || right<=655360)$fatal(1,"small gyro fractions were discarded");
  send(0,0);send(2,655360);gyro=100;
  for(i=0;i<30;i=i+1)begin control_step();if(i%10==0)send(4,action);end
  gyro=0;#1;
  if(left<=655360 || right>=655360)$fatal(1,"positive rotation correction wrong");
  imu_valid=0;repeat(3)@(negedge clk);
  if(mode!=0 || reason!=3)$fatal(1,"gyro heading failed to stop on stale IMU");
  $display("PASS gyro straight: magnetic disagreement, both directions, fractional integration, per-action reset, stale IMU");$finish;
 end
 initial begin #1000000;$fatal(1,"test timeout");end
endmodule
