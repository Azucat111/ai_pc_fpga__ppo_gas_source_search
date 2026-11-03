`timescale 1ns/1ps
module tb_imu_plausibility;
 reg clk=0;always #500 clk=~clk;
 reg rstn=0;reg [31:0] now_ms=0;integer divider=0;
 always @(posedge clk) begin
   if(!rstn)begin divider<=0;now_ms<=0;end
   else if(divider==999)begin divider<=0;now_ms<=now_ms+1;end
   else divider<=divider+1;
 end
 tri1 scl,sda;
 i2c_slave_model #(.ADDRESS(7'h23)) imu(scl,sda);
 i2c_slave_model #(.ADDRESS(7'h48),.ADS(1)) adc(scl,sda);
 wire busy,done,error,valid,fresh;wire [255:0] ps_data;
 wire signed [31:0] yaw,gyro;wire [31:0] stamp,errors,adc_errors,t0,t1;
 wire signed [15:0] a0,a1;wire [1:0] adc_valid;
 sensors #(.CLK_HZ(1000000)) dut(clk,rstn,now_ms,1'b0,7'h77,8'hd0,6'd0,6'd0,256'd0,
  busy,done,error,ps_data,valid,fresh,yaw,gyro,stamp,errors,adc_errors,a0,a1,adc_valid,t0,t1,scl,sda);
 task put_yaw(input [31:0] bits);begin
  @(negedge clk);imu.memory[46]=bits[7:0];imu.memory[47]=bits[15:8];
  imu.memory[48]=bits[23:16];imu.memory[49]=bits[31:24];
 end endtask
 integer saved_stamp,saved_errors,i;
 reg signed [31:0] saved_yaw;
 reg [31:0] ramp[0:11];
 initial begin
  #1;imu.memory[14]=0;imu.memory[15]=0;
  ramp[0]=32'h3fa00000;ramp[1]=32'h3f900000;ramp[2]=32'h3f800000;
  ramp[3]=32'h3f600000;ramp[4]=32'h3f400000;ramp[5]=32'h3f200000;
  ramp[6]=32'h3f000000;ramp[7]=32'h3ec00000;ramp[8]=32'h3e800000;
  ramp[9]=32'h3e000000;ramp[10]=0;ramp[11]=0;
  put_yaw(32'h3fa66666); // 1.3rad, like the real zero-frame incident.
  #3000;@(negedge clk);rstn=1;
  wait(valid);wait(!dut.bus.busy);saved_stamp=stamp;saved_yaw=yaw;saved_errors=errors;
  put_yaw(0);
  wait(now_ms>=saved_stamp+45);#1;
  if(!valid || stamp!=saved_stamp || yaw!=saved_yaw || errors<=saved_errors)
   $fatal(1,"Single zero-angle block must be rejected without refreshing the sample");
  put_yaw(32'h3fa7ae14);wait(stamp>saved_stamp);#1;
  if(yaw<1308 || yaw>1311)$fatal(1,"Good sample after outlier must recover immediately");
  // Real motion can legitimately reach zero; zero itself is not a forbidden value.
  for(i=0;i<12;i=i+1)begin
   saved_stamp=stamp;put_yaw(ramp[i]);wait(stamp>saved_stamp);
  end
  #1;if(yaw!=0)$fatal(1,"A legitimate gradual turn to zero must be accepted");
  // A long gap is handled by freshness upstream, and permits later reacquisition.
  wait(!dut.bus.busy);@(negedge clk);force imu.ack=0;saved_stamp=stamp;
  wait(now_ms>=saved_stamp+450);put_yaw(32'h40466666);release imu.ack;
  wait(stamp>saved_stamp);#1;if(yaw<3098 || yaw>3101)$fatal(1,"Reacquisition after a long gap failed");
  saved_stamp=stamp;put_yaw(32'hc0466666);wait(stamp>saved_stamp);#1;
  if(yaw> -3098 || yaw< -3101)$fatal(1,"Legitimate +pi/-pi crossing must not be an outlier");
  $display("PASS IMU plausibility: zero spike rejection, recovery, legitimate zero, long gap and pi crossing");
  $finish;
 end
 initial begin #2000000000;$fatal(1,"test timeout");end
endmodule
