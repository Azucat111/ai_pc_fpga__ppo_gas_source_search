`timescale 1ns/1ps
module motion(
 input wire clk,rstn,tick,enabled,geometry_valid,
 input wire [31:0] now_ms,boot_id,
 input wire cmd_valid,input wire [255:0] cmd,
 input wire imu_valid,imu_new,input wire [31:0] imu_time,imu_timeout,
 input wire signed [31:0] yaw,gyro,
 input wire [31:0] max_target,heading_kp,heading_kd,max_omega,omega_factor_l,omega_factor_r,
 input wire [31:0] turn_tolerance,rate_tolerance,
 output reg [31:0] mode,action_id,action_start,last_seq,stop_reason,reject_seq,command_status,
 output reg signed [31:0] target_l,target_r
);
 wire [31:0] sequence_number=cmd[63:32];
 wire [31:0] cmd_boot=cmd[95:64],issued=cmd[127:96];
 wire [7:0] requested_mode=cmd[135:128];
 wire [15:0] ttl=cmd[159:144];
 wire signed [31:0] p0=cmd[191:160],p1=cmd[223:192];
 wire fresh_seq=$signed(sequence_number-last_seq)>0;
 wire imu_ok=imu_valid && now_ms-imu_time<=imu_timeout;
 reg [31:0] last_keepalive,lease_ms;
 reg signed [31:0] base_l,base_r,remaining;
 // Integrate mrad/s at the existing 100Hz control tick, keeping 1/100mrad
 // fractions. Both straight and relative turns start at zero for each action;
 // fused magnetic yaw is diagnostic only and cannot end a turn early.
 reg signed [63:0] straight_gyro_sum;
 reg [7:0] settled_ticks;
 reg signed [63:0] error,omega,correction_l,correction_r,t_l,t_r;
 always @* begin
   error=0;omega=0;correction_l=0;correction_r=0;t_l=0;t_r=0;
   if(mode==1) begin t_l=base_l;t_r=base_r;end
   if(mode==2 || mode==3) begin
     error=((mode==2)?64'sd0:$signed(remaining))-(straight_gyro_sum/64'sd100);
     omega=(error*$signed({1'b0,heading_kp})-$signed(gyro)*$signed({1'b0,heading_kd}))>>>8;
     if(omega>$signed({1'b0,max_omega})) omega=$signed({1'b0,max_omega});
     if(omega< -$signed({1'b0,max_omega})) omega= -$signed({1'b0,max_omega});
     correction_l=(omega*$signed({1'b0,omega_factor_l}))/1000;
     correction_r=(omega*$signed({1'b0,omega_factor_r}))/1000;
     t_l=((mode==2)?$signed(base_l):64'sd0)-correction_l;
     t_r=((mode==2)?$signed(base_r):64'sd0)+correction_r;
   end
   if(t_l>$signed({1'b0,max_target})) t_l=$signed({1'b0,max_target});
   if(t_l< -$signed({1'b0,max_target})) t_l= -$signed({1'b0,max_target});
   if(t_r>$signed({1'b0,max_target})) t_r=$signed({1'b0,max_target});
   if(t_r< -$signed({1'b0,max_target})) t_r= -$signed({1'b0,max_target});
   target_l=t_l[31:0];target_r=t_r[31:0];
 end
 task automatic stop(input [31:0] reason);
   begin
     if(mode!=0) begin action_start<=now_ms;action_id<=0;end
     mode<=0;stop_reason<=reason;settled_ticks<=0;
   end
 endtask
 always @(posedge clk) begin
   if(!rstn) begin
     mode<=0;action_id<=0;action_start<=0;last_seq<=0;stop_reason<=1;reject_seq<=0;command_status<=0;
     last_keepalive<=0;lease_ms<=500;base_l<=0;base_r<=0;remaining<=0;settled_ticks<=0;straight_gyro_sum<=0;
   end else begin
     if((mode==2 || mode==3) && tick && imu_ok && enabled)
       straight_gyro_sum<=straight_gyro_sum+$signed(gyro);
     if(mode==3 && tick) begin
       if(error<=$signed({1'b0,turn_tolerance}) && error>= -$signed({1'b0,turn_tolerance}) &&
          $signed(gyro)<=$signed({1'b0,rate_tolerance}) && $signed(gyro)>= -$signed({1'b0,rate_tolerance})) begin
         if(settled_ticks>=19) stop(4);else settled_ticks<=settled_ticks+1;
       end else settled_ticks<=0;
     end
     if(mode!=0 && now_ms-last_keepalive>=lease_ms) stop(2);
     if((mode==2 || mode==3) && !imu_ok) stop(3);
     if(!enabled) stop(1);
     if(cmd_valid) begin
       reject_seq<=sequence_number;
       if(cmd_boot!=boot_id || boot_id==0) command_status<=1;
       else if(requested_mode==0) begin
         stop(0);command_status<=0;
         if(fresh_seq) last_seq<=sequence_number;
       end else if(!fresh_seq) begin
         // Exact retransmission is acknowledged without replaying or extending lease.
         command_status<=(sequence_number==last_seq)?0:2;
       end else if(ttl<100 || ttl>500 || now_ms-issued>ttl) command_status<=3;
       else if(!enabled) command_status<=4;
       else if(requested_mode==4) begin
         if(mode!=0 && p0==action_id && now_ms-last_keepalive<lease_ms) begin
           last_seq<=sequence_number;last_keepalive<=now_ms;lease_ms<=ttl;command_status<=0;
         end else command_status<=5;
       end else if(requested_mode>3) command_status<=6;
       else if(requested_mode>=2 && (!imu_ok || !geometry_valid)) command_status<=7;
       else if((requested_mode==1 || requested_mode==2) &&
         (p0>$signed({1'b0,max_target}) || p0< -$signed({1'b0,max_target}) ||
          p1>$signed({1'b0,max_target}) || p1< -$signed({1'b0,max_target}))) command_status<=8;
       else if(requested_mode==3 && (p0>12566 || p0< -12566 || p1!=0)) command_status<=8;
       else begin
         mode<=requested_mode;action_id<=sequence_number;action_start<=now_ms;
         last_seq<=sequence_number;last_keepalive<=now_ms;lease_ms<=ttl;stop_reason<=0;command_status<=0;
         base_l<=p0;base_r<=p1;remaining<=p0;settled_ticks<=0;straight_gyro_sum<=0;
       end
     end
   end
 end
endmodule
