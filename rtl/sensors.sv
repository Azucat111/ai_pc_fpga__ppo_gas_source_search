`timescale 1ns/1ps
module sensors #(parameter integer CLK_HZ=50000000)(
 input wire clk,rstn,input wire [31:0] now_ms,
 input wire ps_request,input wire [6:0] ps_address,input wire [7:0] ps_register,
 input wire [5:0] ps_wlen,ps_rlen,input wire [255:0] ps_wdata,
 output reg ps_busy,ps_done,ps_error,output reg [255:0] ps_rdata,
 output reg imu_valid,imu_new,output reg signed [31:0] yaw_mrad,gyro_mrad_s,
 output reg [31:0] imu_time,imu_errors,adc_errors,
 output reg signed [15:0] adc0,adc1,output reg [1:0] adc_valid,
 output reg [31:0] adc_time0,adc_time1,
 inout wire scl,sda);
 reg start;
 reg [6:0] addr;
 reg [7:0] regaddr;
 reg [5:0] wlen,rlen;
 reg [255:0] wdata;
 wire busy,done,error;
 wire [255:0] rdata;
 i2c_master #(.CLK_HZ(CLK_HZ)) bus(clk,rstn,start,addr,regaddr,wlen,rlen,wdata,busy,done,error,rdata,scl,sda);
 wire signed [31:0] decoded_yaw;
 wire float_valid;
 float_to_mrad decoder(rdata[95:64],decoded_yaw,float_valid);
 reg [2:0] job;
 reg yaw_pending,ps_pending;
 reg [31:0] imu_poll_time,adc_poll_time,adc_start_time,adc_check_time;
 reg [1:0] adc_state;
 reg adc_channel;
 reg signed [15:0] gyro_raw;
 reg signed [63:0] gyro_product;
 wire [31:0] sample_age_ms=now_ms-imu_time;
 reg signed [63:0] yaw_step;
 reg [63:0] yaw_step_limit;
 wire yaw_step_bad=imu_valid &&
  (yaw_step>$signed(yaw_step_limit) || yaw_step< -$signed(yaw_step_limit));
 reg [6:0] p_addr;
 reg [7:0] p_reg;
 reg [5:0] p_wlen,p_rlen;
 reg [255:0] p_data;
 reg [1:0] next_owner;
 reg [1:0] selected_owner;
 wire imu_due=now_ms-imu_poll_time>=20;
 wire adc_due=adc_state==2 || (adc_state==1 && now_ms-adc_check_time>=10) ||
              (adc_state==0 && now_ms-adc_poll_time>=50);
 // Round-robin across IMU, ADC and PS. A 20ms bus timeout must not make
 // the 20ms IMU job perpetually overdue and starve every other bus user.
 always @* begin
   selected_owner=0;
   case(next_owner)
     0: if(imu_due)selected_owner=1;else if(adc_due)selected_owner=2;else if(ps_pending)selected_owner=3;
     1: if(adc_due)selected_owner=2;else if(ps_pending)selected_owner=3;else if(imu_due)selected_owner=1;
     default: if(ps_pending)selected_owner=3;else if(imu_due)selected_owner=1;else if(adc_due)selected_owner=2;
   endcase
 end
 always @* gyro_product=$signed(gyro_raw)*64'sd1117045; // round(2000/32767*pi/180*1000 * 2^20)
 // Vehicle plausibility screen: 8rad/s plus 40mrad allowance, far above
 // configured car motion. Wrap at pi; a rejected read never renews imu_time.
 // Longer gaps naturally allow reacquisition, while upstream freshness still
 // stops heading control after its configured timeout (normally 150ms).
 always @* begin
   yaw_step=$signed(decoded_yaw)-$signed(yaw_mrad);
   if(yaw_step>3141) yaw_step=yaw_step-6283;
   else if(yaw_step< -3142) yaw_step=yaw_step+6283;
   if(yaw_step>3141) yaw_step=yaw_step-6283;
   else if(yaw_step< -3142) yaw_step=yaw_step+6283;
   yaw_step_limit=64'd40+({32'b0,sample_age_ms}<<3);
 end
 always @(posedge clk) begin
   if(!rstn) begin
     start<=0;addr<=0;regaddr<=0;wlen<=0;rlen<=0;wdata<=0;job<=0;
     ps_busy<=0;ps_done<=0;ps_error<=0;ps_rdata<=0;ps_pending<=0;
     p_addr<=0;p_reg<=0;p_wlen<=0;p_rlen<=0;p_data<=0;
     imu_valid<=0;imu_new<=0;yaw_mrad<=0;gyro_mrad_s<=0;imu_time<=0;imu_errors<=0;
     yaw_pending<=0;imu_poll_time<=0;gyro_raw<=0;
     next_owner<=0;
     adc0<=0;adc1<=0;adc_valid<=0;adc_time0<=0;adc_time1<=0;adc_errors<=0;
     adc_poll_time<=0;adc_start_time<=0;adc_check_time<=0;adc_state<=0;adc_channel<=0;
   end else begin
     start<=0;ps_done<=0;imu_new<=0;
     if(ps_request && !ps_busy) begin
       ps_busy<=1;ps_pending<=1;ps_error<=0;
       p_addr<=ps_address;p_reg<=ps_register;p_wlen<=ps_wlen;p_rlen<=ps_rlen;p_data<=ps_wdata;
     end
     if(done) begin
       job<=0;
       case(job)
         // Validity means a last-good sample exists. On failure keep its data
         // and original timestamp; car_core/motion enforce imu_timeout freshness.
         1: if(error) begin imu_errors<=imu_errors+1;yaw_pending<=0;end
            else begin gyro_raw<=rdata[47:32];yaw_pending<=1;end
         2: begin
           yaw_pending<=0;
           if(error || !float_valid || decoded_yaw>6284 || decoded_yaw< -6284 || yaw_step_bad) begin
             imu_errors<=imu_errors+1;
           end else begin
             yaw_mrad<=decoded_yaw;gyro_mrad_s<=gyro_product>>>20;
             imu_valid<=1;imu_new<=1;imu_time<=now_ms;
           end
         end
         3: if(error) begin adc_valid[adc_channel]<=0;adc_errors<=adc_errors+1;adc_state<=0;end
            else begin adc_state<=1;adc_start_time<=now_ms;adc_check_time<=now_ms;end
         4: if(error || now_ms-adc_start_time>30) begin
              adc_valid[adc_channel]<=0;adc_errors<=adc_errors+1;adc_state<=0;
            end else if(rdata[7]) adc_state<=2;
            else adc_check_time<=now_ms;
         5: begin
           adc_state<=0;adc_channel<=!adc_channel;
           if(error) begin adc_valid[adc_channel]<=0;adc_errors<=adc_errors+1;end
           else begin
             adc_valid[adc_channel]<=1;
             if(adc_channel) begin adc1<={rdata[7:0],rdata[15:8]};adc_time1<=now_ms;end
             else begin adc0<={rdata[7:0],rdata[15:8]};adc_time0<=now_ms;end
           end
         end
         6: begin ps_busy<=0;ps_done<=1;ps_error<=error;ps_rdata<=rdata;end
       endcase
     end
     if(job==0 && !busy && !start && !done) begin
       wdata<=0;wlen<=0;rlen<=0;
       if(yaw_pending) begin
         job<=2;addr<=7'h23;regaddr<=8'h26;rlen<=12;start<=1;
       end else if(selected_owner==1) begin
         next_owner<=1;
         imu_poll_time<=now_ms;job<=1;addr<=7'h23;regaddr<=8'h0a;rlen<=6;start<=1;
       end else if(selected_owner==2 && adc_state==2) begin
         next_owner<=2;
         job<=5;addr<=7'h48;regaddr<=0;rlen<=2;start<=1;
       end else if(selected_owner==2 && adc_state==1) begin
         next_owner<=2;
         job<=4;addr<=7'h48;regaddr<=1;rlen<=2;start<=1;
       end else if(selected_owner==2 && adc_state==0) begin
         next_owner<=2;
         adc_poll_time<=now_ms;job<=3;addr<=7'h48;regaddr<=1;wlen<=2;
         wdata[15:0]<=adc_channel ? 16'h83d1:16'h83c1;start<=1;
       end else if(selected_owner==3) begin
         next_owner<=0;
         job<=6;addr<=p_addr;regaddr<=p_reg;wlen<=p_wlen;rlen<=p_rlen;wdata<=p_data;
         ps_pending<=0;start<=1;
       end
     end
   end
 end
endmodule
