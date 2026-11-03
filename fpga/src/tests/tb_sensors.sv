`timescale 1ns/1ps
module tb_sensors;
 reg clk=0;always #500 clk=~clk;
 reg rstn=0;reg[31:0] now_ms=0;
 integer div=0;
 always @(posedge clk)if(rstn)begin if(div==999)begin div<=0;now_ms<=now_ms+1;end else div<=div+1;end
 tri1 scl,sda;
 i2c_slave_model #(.ADDRESS(7'h23)) imu(scl,sda);
 i2c_slave_model #(.ADDRESS(7'h48),.ADS(1)) adc(scl,sda);
 i2c_slave_model #(.ADDRESS(7'h77)) bme(scl,sda);
 reg request=0;
 wire ps_busy,ps_done,ps_error;wire[255:0] ps_data;
 wire imu_valid,imu_new;wire signed[31:0] yaw,gyro;
 wire [31:0] imu_time,imu_errors,adc_errors,t0,t1;
 wire signed[15:0] a0,a1;wire[1:0] adc_valid;
 sensors #(.CLK_HZ(1000000)) dut(clk,rstn,now_ms,request,7'h77,8'hd0,6'd0,6'd1,256'd0,
 ps_busy,ps_done,ps_error,ps_data,imu_valid,imu_new,yaw,gyro,imu_time,imu_errors,adc_errors,a0,a1,adc_valid,t0,t1,scl,sda);
 initial begin
   #1;
   imu.memory[14]=0;imu.memory[15]=16;
   imu.memory[46]=8'hdb;imu.memory[47]=8'h0f;imu.memory[48]=8'hc9;imu.memory[49]=8'h3f;
   adc.memory[0]=8'h12;adc.memory[1]=8'h34;bme.memory[208]=8'h61;
   #3000;@(negedge clk);rstn=1;
   wait(now_ms==21);@(negedge clk);request=1;@(negedge clk);request=0;
   wait(ps_done);#1;if(ps_error || ps_data[7:0]!=8'h61)$fatal(1,"PS mailbox arbitration/read");
   wait(now_ms==150);#1;
   if(!imu_valid || yaw!=1570 || gyro!=4363)$fatal(1,"IMU decode: %d %d valid=%d",yaw,gyro,imu_valid);
   if(adc_valid!=3 || a0!=16'h1234 || a1!=16'h1234)$fatal(1,"ADC scheduler %d %h %h",adc_valid,a0,a1);
   if(imu_errors || adc_errors)$fatal(1,"unexpected scheduler errors %d %d",imu_errors,adc_errors);
   $display("PASS sensors: autonomous IMU + both ADS1115 channels + concurrent PS mailbox");$finish;
 end
 initial begin #200000000;$fatal(1,"test timeout");end
endmodule
