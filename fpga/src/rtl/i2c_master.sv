`timescale 1ns/1ps
// Single-master, 8-bit register address, up to 32 write/read bytes.
// Read transactions use repeated START. True open-drain pins, stretch aware.
module i2c_master #(parameter integer CLK_HZ=50000000,I2C_HZ=100000)(
 input wire clk,rstn,start,input wire [6:0] address,input wire [7:0] register_address,
 input wire [5:0] write_length,read_length,input wire [255:0] write_data,
 output reg busy,done,error,output reg [255:0] read_data,
 inout wire scl,sda);
 localparam integer HALF=CLK_HZ/(2*I2C_HZ),TIMEOUT=CLK_HZ/50;
 localparam IDLE=0,START_WAIT=1,START_HOLD=2,BIT_LOW=3,BIT_WAIT=4,BIT_HIGH=5,
 ACK_LOW=6,ACK_WAIT=7,ACK_HIGH=8,RESTART_LOW=9,STOP_LOW=10,STOP_WAIT=11,
 STOP_HIGH=12,STOP_RELEASE=13,RECOVER_LOW=14,RECOVER_WAIT=15,RECOVER_HIGH=16;
 reg [4:0] state;
 reg recovering;
 reg [3:0] recovery_pulses;
 reg [2:0] stage;
 reg [6:0] addr;
 reg [7:0] reg_addr,tx_byte,rx_byte;
 reg [255:0] tx_data;
 reg [5:0] tx_len,rx_len,byte_index;
 integer timer,elapsed,bit_index;
 reg scl_low,sda_low;
 (* ASYNC_REG="TRUE" *) reg [1:0] scl_sync,sda_sync;
 wire reading=stage==4;
 assign scl=scl_low?1'b0:1'bz;
 assign sda=sda_low?1'b0:1'bz;
 always @* begin
   scl_low=0;sda_low=0;
   case(state)
     START_HOLD: sda_low=1;
     BIT_LOW,BIT_WAIT,BIT_HIGH: begin
       scl_low=state==BIT_LOW;sda_low=!reading && !tx_byte[bit_index];end
     ACK_LOW,ACK_WAIT,ACK_HIGH: begin
       scl_low=state==ACK_LOW;sda_low=reading && byte_index+1<rx_len;end
     RESTART_LOW: scl_low=1;
     STOP_LOW: begin scl_low=1;sda_low=1;end
     STOP_WAIT,STOP_HIGH: sda_low=1;
     RECOVER_LOW: scl_low=1;
     default: begin scl_low=0;sda_low=0;end
   endcase
 end
 always @(posedge clk) begin
   if(!rstn) begin
     state<=IDLE;busy<=0;done<=0;error<=0;read_data<=0;timer<=0;elapsed<=0;
     scl_sync<=3;sda_sync<=3;stage<=0;addr<=0;reg_addr<=0;tx_byte<=0;rx_byte<=0;
     tx_data<=0;tx_len<=0;rx_len<=0;byte_index<=0;bit_index<=7;
     recovering<=0;recovery_pulses<=0;
   end else begin
     scl_sync<={scl_sync[0],scl};sda_sync<={sda_sync[0],sda};done<=0;
     if(timer>0) timer<=timer-1;
     if(busy) elapsed<=elapsed+1;
     // A timeout can interrupt a slave while it is transmitting a zero. Merely
     // releasing SDA then retrying START leaves that slave holding SDA forever.
     // Clock nine bits with SDA released, then issue STOP. The failed transfer
     // still reports error; recovery never turns partial bytes into valid data.
     if(busy && elapsed>=TIMEOUT) begin
       error<=1;
       if(recovering)begin state<=IDLE;busy<=0;done<=1;end
       else begin state<=RECOVER_LOW;timer<=HALF;elapsed<=0;recovering<=1;recovery_pulses<=0;end
     end
     else case(state)
       IDLE: if(start) begin
         addr<=address;reg_addr<=register_address;tx_len<=write_length;rx_len<=read_length;
         tx_data<=write_data;read_data<=0;error<=0;busy<=1;elapsed<=0;
         byte_index<=0;stage<=0;tx_byte<={address,1'b0};timer<=HALF;state<=START_WAIT;
         recovering<=0;recovery_pulses<=0;
         if(write_length>32 || read_length>32) begin busy<=0;done<=1;error<=1;state<=IDLE;end
       end
       START_WAIT: begin
         if(scl_sync[1] && !sda_sync[1])begin
           error<=1;recovering<=1;recovery_pulses<=0;elapsed<=0;timer<=HALF;state<=RECOVER_LOW;
         end
         else if(!scl_sync[1]) timer<=HALF;
         else if(timer==0) begin state<=START_HOLD;timer<=HALF;end
       end
       START_HOLD: if(timer==0) begin state<=BIT_LOW;bit_index<=7;timer<=HALF;end
       BIT_LOW: if(timer==0) state<=BIT_WAIT;
       BIT_WAIT: if(scl_sync[1]) begin state<=BIT_HIGH;timer<=HALF;end
       BIT_HIGH: if(timer==0) begin
         if(reading) rx_byte[bit_index]<=sda_sync[1];
         if(bit_index==0) state<=ACK_LOW;
         else begin bit_index<=bit_index-1;state<=BIT_LOW;end
         timer<=HALF;
       end
       ACK_LOW: if(timer==0) state<=ACK_WAIT;
       ACK_WAIT: if(scl_sync[1]) begin state<=ACK_HIGH;timer<=HALF;end
       ACK_HIGH: if(timer==0) begin
         bit_index<=7;timer<=HALF;
         if(!reading && sda_sync[1]) begin error<=1;state<=STOP_LOW;end
         else case(stage)
           0: begin stage<=1;tx_byte<=reg_addr;state<=BIT_LOW;end
           1: begin
             if(tx_len!=0) begin stage<=2;byte_index<=0;tx_byte<=tx_data[7:0];state<=BIT_LOW;end
             else if(rx_len!=0) begin stage<=3;tx_byte<={addr,1'b1};state<=RESTART_LOW;end
             else state<=STOP_LOW;
           end
           2: begin
             if(byte_index+1<tx_len) begin byte_index<=byte_index+1;tx_byte<=tx_data[(byte_index+1)*8+:8];state<=BIT_LOW;end
             else if(rx_len!=0) begin stage<=3;tx_byte<={addr,1'b1};state<=RESTART_LOW;end
             else state<=STOP_LOW;
           end
           3: begin stage<=4;byte_index<=0;rx_byte<=0;state<=BIT_LOW;end
           4: begin
             read_data[byte_index*8+:8]<=rx_byte;
             if(byte_index+1<rx_len) begin byte_index<=byte_index+1;state<=BIT_LOW;end
             else state<=STOP_LOW;
           end
           default: state<=STOP_LOW;
         endcase
       end
       RESTART_LOW: if(timer==0) begin state<=START_WAIT;timer<=HALF;end
       STOP_LOW: if(timer==0) state<=STOP_WAIT;
       STOP_WAIT: if(scl_sync[1]) begin state<=STOP_HIGH;timer<=HALF;end
       STOP_HIGH: if(timer==0) begin state<=STOP_RELEASE;timer<=HALF;end
       STOP_RELEASE: if(timer==0) begin state<=IDLE;busy<=0;done<=1;end
       RECOVER_LOW: if(timer==0)state<=RECOVER_WAIT;
       RECOVER_WAIT: if(scl_sync[1])begin state<=RECOVER_HIGH;timer<=HALF;end
       RECOVER_HIGH: if(timer==0)begin
         timer<=HALF;
         if(recovery_pulses==8)state<=STOP_LOW;
         else begin recovery_pulses<=recovery_pulses+1;state<=RECOVER_LOW;end
       end
       default: state<=IDLE;
     endcase
   end
 end
endmodule
