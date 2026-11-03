`timescale 1ns/1ps
module tb_motion;
 reg clk=0;always #5 clk=~clk;
 reg rstn=0,tick=0,enabled=0,geo=1,valid=0,imu_valid=1,imu_new=0;
 reg [31:0] now_ms=0,imu_time=0;
 reg signed [31:0] yaw=0,gyro=0;
 reg [255:0] cmd=0;
 wire [31:0] mode,action,start,last_seq,reason,reject,status;
 wire signed [31:0] left,right;
 motion dut(clk,rstn,tick,enabled,geo,now_ms,32'h12345678,valid,cmd,
 imu_valid,imu_new,imu_time,32'd150,yaw,gyro,32'd2621440,32'd512,32'd128,32'd600,
 32'd655360,32'd655360,32'd20,32'd80,mode,action,start,last_seq,reason,reject,status,left,right);
 task send(input [31:0] seq,input [7:0] m,input signed [31:0] p0,input signed [31:0] p1,input [31:0] issued);
   begin @(negedge clk);cmd=0;cmd[63:32]=seq;cmd[95:64]=32'h12345678;cmd[127:96]=issued;
   cmd[135:128]=m;cmd[159:144]=500;cmd[191:160]=p0;cmd[223:192]=p1;valid=1;
   @(negedge clk);valid=0;#1;end
 endtask
 task sample(input signed [31:0] y,input signed [31:0] g);
   begin @(negedge clk);yaw=y;gyro=g;imu_time=now_ms;imu_new=1;
   @(negedge clk);imu_new=0;end
 endtask
 initial begin
   #20;@(negedge clk);rstn=1;enabled=1;
   send(1,1,65536,-65536,0);
   if(mode!=1 || left!=65536 || right!=-65536)$fatal(1,"wheel command");
   now_ms=100;send(2,4,1,0,100);
   if(action!=1 || start!=0 || last_seq!=2)$fatal(1,"heartbeat changed action");
   now_ms=200;send(2,4,1,0,100); // duplicate must NOT renew lease
   now_ms=601;repeat(3)@(negedge clk);
   if(mode!=0 || reason!=2)$fatal(1,"watchdog/duplicate failure");
   send(3,4,1,0,601);if(mode!=0 || status!=5)$fatal(1,"heartbeat resurrected action");
   send(4,1,1,1,0);if(status!=3 || mode!=0)$fatal(1,"expired command accepted");
   sample(3130,0);send(5,2,0,0,601);sample(-3130,0);#1;
   if(left!=0 || right!=0)$fatal(1,"magnetic yaw affected gyro straight heading");
   send(6,0,0,0,0);
   sample(0,0);send(7,3,4712,0,601); // +270 degrees, must not choose -90
   if(right<=0 || left>=0)$fatal(1,"long turn wrong direction");
   sample(-1571,6000);
   repeat(78)begin @(negedge clk);tick=1;@(negedge clk);tick=0;end
   sample(-1571,3200);@(negedge clk);tick=1;@(negedge clk);tick=0;sample(-1571,0);
   repeat(22)begin @(negedge clk);tick=1;@(negedge clk);tick=0;end
   if(mode!=0 || reason!=4)$fatal(1,"relative +270 completion: %d",dut.remaining);
   sample(0,0);send(8,3,-1571,0,601);
   if(right>=0 || left<=0)$fatal(1,"negative turn wrong direction");
   sample(1571,-1571);repeat(100)begin @(negedge clk);tick=1;@(negedge clk);tick=0;end
   sample(1571,1000);repeat(1)begin @(negedge clk);tick=1;@(negedge clk);tick=0;end
   if(mode==0)$fatal(1,"turn completed while still rotating");
   sample(-1571,0);repeat(22)begin @(negedge clk);tick=1;@(negedge clk);tick=0;end
   if(mode!=0 || reason!=4)$fatal(1,"negative turn completion");
   send(9,2,0,0,601);imu_valid=0;repeat(3)@(negedge clk);
   if(mode!=0 || reason!=3)$fatal(1,"IMU loss not stopping heading mode");
   send(10,1,1,1,601);enabled=0;repeat(3)@(negedge clk);
   if(mode!=0 || reason!=1)$fatal(1,"disable not stopping");
   $display("PASS motion: direction, magnetic independence, long turn, duplicate, expiry, watchdog, stale IMU, disable");$finish;
 end
 initial begin #100000;$fatal(1,"test timeout");end
endmodule
