`timescale 1ns/1ps
module car_core #(parameter integer CLK_HZ=50000000)(
 input wire clk,rstn,wr_en,input wire [11:0] wr_addr,rd_addr,
 input wire [31:0] wr_data,input wire [3:0] wr_strb,output reg [31:0] rd_data,
 input wire enc_m1_a,enc_m1_b,enc_m2_a,enc_m2_b,esp_rx,
 output wire motor_a_in1,motor_a_in2,motor_b_in1,motor_b_in2,esp_tx,
 inout wire i2c_scl,i2c_sda,output wire source_led);
 reg [31:0] cfg[0:63];
 integer ms_div,control_div,telemetry_div,i,j;
 reg ms_tick,control_tick,telemetry_tick;
 reg [31:0] now_ms;
 wire enabled=cfg[5][0];
 wire geometry_valid=cfg[24][0] && cfg[16]!=0 && cfg[17]!=0 && cfg[18]!=0;
 wire signed [63:0] m1_count,m2_count;
 wire [31:0] enc_error1,enc_error2;
 encoder #(.FILTER_CYCLES(CLK_HZ/1000000)) e1(clk,rstn,enc_m1_a,enc_m1_b,cfg[6][1],m1_count,enc_error1);
 encoder #(.FILTER_CYCLES(CLK_HZ/1000000)) e2(clk,rstn,enc_m2_a,enc_m2_b,cfg[6][2],m2_count,enc_error2);
 wire signed [63:0] left_count=cfg[6][0]?m2_count:m1_count;
 wire signed [63:0] right_count=cfg[6][0]?m1_count:m2_count;
 reg signed [63:0] prev_l,prev_r,telem_prev_l,telem_prev_r;
 wire signed [63:0] delta_l=left_count-prev_l,delta_r=right_count-prev_r;
 reg signed [31:0] measured_l,measured_r;
 wire signed [63:0] increment_l=delta_l*$signed({1'b0,cfg[16]});
 wire signed [63:0] increment_r=delta_r*$signed({1'b0,cfg[17]});
 wire signed [63:0] center_increment=(increment_l+increment_r)>>>1;
 reg signed [63:0] distance_l,distance_r,travelled;
 reg signed [63:0] velocity_l,velocity_r,angular;
 always @* begin
   velocity_l=($signed(measured_l)*$signed({1'b0,cfg[16]})*64'sd100)>>>32;
   velocity_r=($signed(measured_r)*$signed({1'b0,cfg[17]})*64'sd100)>>>32;
   angular=((velocity_r-velocity_l)*$signed({1'b0,cfg[18]}))>>>24;
 end

 reg ps_req,ps_active,ps_complete,ps_result_error;
 reg [6:0] ps_addr;
 reg [7:0] ps_reg;
 reg [5:0] ps_wlen,ps_rlen;
 reg [255:0] ps_wdata,ps_result;
 wire ps_busy,ps_done,ps_error;
 wire [255:0] ps_rdata;
 wire imu_valid,imu_new;
 wire signed [31:0] imu_yaw_raw,imu_gyro_raw;
 wire [31:0] imu_time,imu_errors,adc_errors,adc_time0,adc_time1;
 wire signed [15:0] adc0,adc1;
 wire [1:0] adc_valid;
 sensors #(.CLK_HZ(CLK_HZ)) sensor_bus(clk,rstn,now_ms,
  ps_req,ps_addr,ps_reg,ps_wlen,ps_rlen,ps_wdata,ps_busy,ps_done,ps_error,ps_rdata,
  imu_valid,imu_new,imu_yaw_raw,imu_gyro_raw,imu_time,imu_errors,adc_errors,
  adc0,adc1,adc_valid,adc_time0,adc_time1,i2c_scl,i2c_sda);
 wire [31:0] tgs_rd_data;
 tgs_features tgs_pipeline(clk,rstn,now_ms,adc0,adc1,adc_valid,adc_time0,adc_time1,
   wr_en,wr_addr,rd_addr,wr_data,wr_strb,tgs_rd_data);
 reg signed [31:0] yaw,gyro;
 reg signed [63:0] yaw_temp;
 always @* begin
   yaw_temp=(cfg[21][0]?-$signed(imu_yaw_raw):$signed(imu_yaw_raw))+$signed(cfg[22]);
   if(yaw_temp>3141) yaw_temp=yaw_temp-6283;
   if(yaw_temp>3141) yaw_temp=yaw_temp-6283;
   if(yaw_temp< -3142) yaw_temp=yaw_temp+6283;
   if(yaw_temp< -3142) yaw_temp=yaw_temp+6283;
   yaw=yaw_temp[31:0];gyro=cfg[21][1]?-$signed(imu_gyro_raw):$signed(imu_gyro_raw);
 end
 wire [7:0] rx_data,tx_data;
 wire rx_valid,tx_ready,tx_valid,tx_busy;
 reg tx_start;
 wire main_valid,gas_valid_tx,gas_busy,gas_start,gas_mode;
 wire [7:0] main_data,gas_data;
 wire [31:0] gas_rd_data;reg gas_slot_used;
 wire event_busy,event_start,event_tx_valid,event_confirmed,event_valid;
 wire [7:0] event_data;wire [31:0] event_last_seq;reg event_slot_used;
 wire event_slot=!tx_busy && !tx_start && !gas_busy && !telemetry_tick && !event_slot_used && telemetry_div>=12 && telemetry_div<=14;
 wire gas_slot=!event_busy && !tx_busy && !tx_start && !telemetry_tick && !gas_slot_used && telemetry_div>=6 && telemetry_div<=10;
 assign tx_valid=event_busy?event_tx_valid:(gas_busy?gas_valid_tx:main_valid);
 assign tx_data=event_busy?event_data:(gas_busy?gas_data:main_data);
 gas10_mailbox gasbox(clk,rstn,wr_en,wr_addr,rd_addr,wr_data,wr_strb,cfg[4],now_ms,gas_slot,
  tx_ready && gas_busy,gas_rd_data,gas_valid_tx,gas_data,gas_busy,gas_start,gas_mode);
 wire [31:0] framing_errors,crc_errors;
 wire cmd_valid;
 wire [255:0] cmd;
 uart_rx #(.CLK_HZ(CLK_HZ)) ur(clk,rstn,esp_rx,rx_data,rx_valid,framing_errors);
 event_frame_rx fr(clk,rstn,ms_tick,rx_valid,rx_data,cmd_valid,cmd,crc_errors,event_valid);
 source_indicator indicator(clk,rstn,ms_tick,cfg[4],now_ms,event_valid,cmd,event_slot,
  tx_ready && event_busy,event_tx_valid,event_data,event_busy,event_start,event_confirmed,source_led,event_last_seq);
 uart_tx #(.CLK_HZ(CLK_HZ)) ut(clk,rstn,tx_valid,tx_data,tx_ready,esp_tx);
 wire [31:0] mode,action_id,action_start,last_seq,stop_reason,reject_seq,command_status;
 wire signed [31:0] target_l,target_r;
 motion controller(clk,rstn,control_tick,enabled,geometry_valid && cfg[24][1],now_ms,cfg[4],cmd_valid,cmd,
  imu_valid,imu_new,imu_time,cfg[23],yaw,gyro,cfg[11],cfg[12],cfg[13],cfg[14],cfg[19],cfg[20],
  {16'b0,cfg[15][15:0]},{16'b0,cfg[15][31:16]},mode,action_id,action_start,last_seq,stop_reason,reject_seq,command_status,target_l,target_r);
 wire drive=enabled && mode!=0;
 wire signed [15:0] duty_l,duty_r;
 wheel_pi pl(clk,rstn,control_tick,drive,target_l,measured_l,cfg[7],cfg[8],cfg[10],cfg[9][15:0],cfg[9][31:16],duty_l);
 wheel_pi pr(clk,rstn,control_tick,drive,target_r,measured_r,cfg[7],cfg[8],cfg[10],cfg[9][15:0],cfg[9][31:16],duty_r);
 wire signed [15:0] duty_m1=cfg[6][0]?duty_r:duty_l;
 wire signed [15:0] duty_m2=cfg[6][0]?duty_l:duty_r;
 motor_pwm #(.CLK_HZ(CLK_HZ)) pwm1(clk,rstn,drive,cfg[6][3],duty_m1,motor_a_in1,motor_a_in2);
 motor_pwm #(.CLK_HZ(CLK_HZ)) pwm2(clk,rstn,drive,cfg[6][4],duty_m2,motor_b_in1,motor_b_in2);

 // Read-only motor diagnostics. Counters observe the actual output logic;
 // retained targets/duty allow inspection after STOP without extending motion.
 wire [3:0] debug_motor_pins={motor_b_in2,motor_b_in1,motor_a_in2,motor_a_in1};
 reg [3:0] debug_previous_pins;
 reg [31:0] debug_control_ticks,debug_motion_mode,debug_motion_p0,debug_motion_p1;
 reg [31:0] debug_active_target_l,debug_active_target_r;
 reg [31:0] debug_rising_edges[0:3];
 reg [31:0] debug_last_duty;
 integer debug_index;
 always @(posedge clk) begin
   if(!rstn) begin
     debug_previous_pins<=0;debug_control_ticks<=0;debug_motion_mode<=0;
     debug_motion_p0<=0;debug_motion_p1<=0;debug_last_duty<=0;
     debug_active_target_l<=0;debug_active_target_r<=0;
     for(debug_index=0;debug_index<4;debug_index=debug_index+1) debug_rising_edges[debug_index]<=0;
   end else begin
     debug_previous_pins<=debug_motor_pins;
     for(debug_index=0;debug_index<4;debug_index=debug_index+1)
       if(debug_motor_pins[debug_index] && !debug_previous_pins[debug_index])
         debug_rising_edges[debug_index]<=debug_rising_edges[debug_index]+1;
     if(control_tick) begin
       debug_control_ticks<=debug_control_ticks+1;
       if(drive) begin debug_active_target_l<=target_l;debug_active_target_r<=target_r;end
     end
     if(cmd_valid && cmd[135:128]>=1 && cmd[135:128]<=3) begin
       debug_motion_mode<={24'b0,cmd[135:128]};
       debug_motion_p0<=cmd[191:160];debug_motion_p1<=cmd[223:192];
     end
     if(duty_l!=0 || duty_r!=0) debug_last_duty<={duty_r,duty_l};
   end
 end

 reg [31:0] bme_values[0:3];
 reg [31:0] bme_time,bme_flags;
 wire imu_fresh=imu_valid && now_ms-imu_time<=cfg[23];
 wire adc_fresh0=adc_valid[0] && now_ms-adc_time0<=300;
 wire adc_fresh1=adc_valid[1] && now_ms-adc_time1<=300;
 wire bme_fresh=bme_flags[0] && now_ms-bme_time<=5000;
 wire [31:0] flags={21'b0,1'b1,gas_mode,1'b1,1'b1,cfg[24][1],bme_fresh,geometry_valid,adc_fresh1,adc_fresh0,imu_fresh,enabled};
 reg [1439:0] payload,snapshot;
 reg [31:0] snapshot_seq,snapshot_time;
 frame_tx ft(clk,rstn,tx_start,snapshot,snapshot_seq,tx_ready && !gas_busy && !event_busy,main_valid,main_data,tx_busy);
 always @* begin
   payload=0;
   payload[0*32+:32]=cfg[4];payload[1*32+:32]=now_ms;
   payload[2*32+:32]=last_seq;payload[3*32+:32]=action_id;
   payload[4*32+:32]=mode;payload[5*32+:32]=now_ms-action_start;
   payload[6*32+:32]=stop_reason;payload[7*32+:32]=flags;
   payload[8*32+:64]=left_count;payload[10*32+:64]=right_count;
   payload[12*32+:32]=left_count-telem_prev_l;payload[13*32+:32]=right_count-telem_prev_r;
   payload[14*32+:32]=measured_l;payload[15*32+:32]=measured_r;
   payload[16*32+:32]=yaw;payload[17*32+:32]=gyro;
   payload[18*32+:32]=now_ms-imu_time;
   payload[19*32+:32]={{16{adc0[15]}},adc0};payload[20*32+:32]={{16{adc1[15]}},adc1};
   payload[21*32+:32]=now_ms-adc_time0;payload[22*32+:32]=now_ms-adc_time1;
   payload[23*32+:32]=bme_values[0];payload[24*32+:32]=bme_values[1];
   payload[25*32+:32]=bme_values[2];payload[26*32+:32]=bme_values[3];
   payload[27*32+:32]=now_ms-bme_time;payload[28*32+:32]=bme_flags;
   payload[29*32+:64]=distance_l;payload[31*32+:64]=distance_r;
   payload[33*32+:32]=(velocity_l+velocity_r)>>>1;
   payload[34*32+:32]=angular;payload[35*32+:64]=travelled;
   payload[37*32+:32]=now_ms-snapshot_time;
   payload[38*32+:32]=command_status;payload[39*32+:32]=imu_errors;
   payload[40*32+:32]=adc_errors;payload[41*32+:32]=crc_errors+framing_errors;
   payload[42*32+:32]=reject_seq;payload[43*32+:32]=enc_error1;payload[44*32+:32]=enc_error2;
 end
 always @* begin
   rd_data=0;
   if(rd_addr<12'h100) rd_data=cfg[rd_addr[7:2]];
   case(rd_addr)
     12'h000: rd_data=32'h43415231;
     12'h004: rd_data=32'h00010000;
     12'h008: rd_data=now_ms;
     12'h00c: rd_data=flags;
     12'h064: rd_data=32'h44424731;
     12'h068: rd_data=CLK_HZ;
     12'h06c: rd_data=debug_control_ticks;
     12'h070: rd_data=target_l;
     12'h074: rd_data=target_r;
     12'h078: rd_data={duty_r,duty_l};
     12'h07c: rd_data={27'b0,drive,debug_motor_pins};
     12'h0a0: rd_data=debug_motion_mode;
     12'h0a4: rd_data=debug_motion_p0;
     12'h0a8: rd_data=debug_motion_p1;
     12'h0ac: rd_data=debug_last_duty;
     12'h0b0: rd_data=debug_rising_edges[0];
     12'h0b4: rd_data=debug_rising_edges[1];
     12'h0b8: rd_data=debug_rising_edges[2];
     12'h0bc: rd_data=debug_rising_edges[3];
     12'h0c0: rd_data=debug_active_target_l;
     12'h0c4: rd_data=debug_active_target_r;
     12'h0c8: rd_data=32'd250; // A4950 standby wake interval, microseconds (v0.1.4+).
     12'h0cc: rd_data=32'h494d5532; // IMU2: last-good sample plus wrapped yaw plausibility screen.
     12'h0d0: rd_data=32'h47595231; // GYR1: straight control uses per-action gyro integration.
     12'h0d4: rd_data=32'h47595232; // GYR2: relative turns also use per-action gyro integration.
     12'h0d8: rd_data=32'h47415331; // GAS1: independent GAS10 mailbox/0x82 frames.
     12'h0e0: rd_data=32'h4c454431;
     12'h4d0: rd_data={30'b0,source_led,event_confirmed};
     12'h4d4: rd_data=event_last_seq;
     12'h0dc: rd_data=32'h49324332; // I2C2: bounded bus clear and fair sensor arbitration.
     12'h110: rd_data={29'b0,ps_result_error,ps_complete,ps_active};
     12'h104: rd_data={25'b0,ps_addr};
     12'h108: rd_data={24'b0,ps_reg};
     12'h10c: rd_data={18'b0,ps_rlen,2'b0,ps_wlen};
     12'h1f0: rd_data=snapshot_seq;
   endcase
   if(rd_addr>=12'h500 && rd_addr<12'h5a0) rd_data=tgs_rd_data;
   if(rd_addr>=12'h140 && rd_addr<12'h160) rd_data=ps_result[(rd_addr-12'h140)*8+:32];
   if(rd_addr>=12'h300 && rd_addr<12'h4b4)rd_data=gas_rd_data;
   if(rd_addr>=12'h200 && rd_addr<12'h2b4) rd_data=snapshot[(rd_addr-12'h200)*8+:32];
 end
 always @(posedge clk) begin
   if(!rstn) begin
     for(i=0;i<64;i=i+1) cfg[i]<=0;
     cfg[7]<=2560;cfg[8]<=256;cfg[9]<=250;cfg[10]<=4*65536;cfg[11]<=40*65536;
     cfg[12]<=512;cfg[13]<=128;cfg[14]<=600;cfg[15]<={16'd80,16'd20};cfg[23]<=150;
     event_slot_used<=0;gas_slot_used<=0;ms_div<=0;control_div<=0;telemetry_div<=0;now_ms<=0;ms_tick<=0;control_tick<=0;telemetry_tick<=0;
     prev_l<=0;prev_r<=0;telem_prev_l<=0;telem_prev_r<=0;measured_l<=0;measured_r<=0;
     distance_l<=0;distance_r<=0;travelled<=0;
     ps_req<=0;ps_active<=0;ps_complete<=0;ps_result_error<=0;ps_result<=0;
     ps_addr<=0;ps_reg<=0;ps_wlen<=0;ps_rlen<=0;ps_wdata<=0;
     for(i=0;i<4;i=i+1) bme_values[i]<=0;
     bme_time<=0;bme_flags<=0;snapshot<=0;snapshot_seq<=0;snapshot_time<=0;tx_start<=0;
   end else begin
     ms_tick<=0;control_tick<=0;telemetry_tick<=0;ps_req<=0;tx_start<=0;
     if(ms_div==CLK_HZ/1000-1) begin ms_div<=0;now_ms<=now_ms+1;ms_tick<=1;
       if(control_div==9) begin control_div<=0;control_tick<=1;end else control_div<=control_div+1;
       if(telemetry_div==19) begin telemetry_div<=0;telemetry_tick<=1;end else telemetry_div<=telemetry_div+1;
     end else ms_div<=ms_div+1;
     if(telemetry_tick)begin gas_slot_used<=0;event_slot_used<=0;end
     if(event_start)event_slot_used<=1;
     if(gas_start)gas_slot_used<=1;
     if(control_tick) begin
       prev_l<=left_count;prev_r<=right_count;
       measured_l<=measured_l+(((delta_l<<<16)-$signed(measured_l))>>>2);
       measured_r<=measured_r+(((delta_r<<<16)-$signed(measured_r))>>>2);
       if(geometry_valid) begin
         distance_l<=distance_l+increment_l;distance_r<=distance_r+increment_r;
         travelled<=travelled+(center_increment[63]?-center_increment:center_increment);
       end
     end
     if(telemetry_tick && !tx_busy && !gas_busy && !gas_start && !event_busy && !event_start) begin
       snapshot<=payload;snapshot_seq<=snapshot_seq+1;snapshot_time<=now_ms;tx_start<=1;
       telem_prev_l<=left_count;telem_prev_r<=right_count;
     end
     if(ps_done) begin ps_active<=0;ps_complete<=1;ps_result_error<=ps_error;ps_result<=ps_rdata;end
     if(wr_en && wr_addr[1:0]==0) begin
       if(wr_addr<12'h100 && ((wr_addr>=12'h010 && wr_addr<=12'h060 && !enabled) || wr_addr==12'h014 ||
          (wr_addr>=12'h080 && wr_addr<=12'h098))) begin
         for(j=0;j<4;j=j+1) if(wr_strb[j]) cfg[wr_addr[7:2]][j*8+:8]<=wr_data[j*8+:8];
       end
       if(wr_addr==12'h098 && wr_strb[0]) begin
         for(j=0;j<4;j=j+1) bme_values[j]<=cfg[32+j];
         bme_flags<=wr_data&7;bme_time<=now_ms;
       end
       if(!ps_active && wr_strb==4'hf) begin
         case(wr_addr)
           12'h100: if(wr_data[0]) begin ps_req<=1;ps_active<=1;ps_complete<=0;ps_result_error<=0;end
           12'h104: ps_addr<=wr_data[6:0];
           12'h108: ps_reg<=wr_data[7:0];
           12'h10c: begin ps_wlen<=wr_data[5:0];ps_rlen<=wr_data[13:8];end
         endcase
         if(wr_addr>=12'h120 && wr_addr<12'h140) ps_wdata[(wr_addr-12'h120)*8+:32]<=wr_data;
       end
     end
   end
 end
endmodule
