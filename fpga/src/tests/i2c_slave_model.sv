`timescale 1ns/1ps
module i2c_slave_model #(parameter [6:0] ADDRESS=7'h23,parameter integer ADS=0)(inout wire scl,sda);
 reg low=0;
 assign sda=low?1'b0:1'bz;
 reg [7:0] memory[0:255];
 integer state=0,next_state=0,bits=0,pointer=0;
 reg [7:0] incoming=0;
 reg ack=0,ack_clocked=0,master_ack=0;
 integer i;
 initial for(i=0;i<256;i=i+1)memory[i]=i;
 // 1 address, 2 register, 3 data write, 4 ACK, 5 data read, 6 master ACK.
 always @(negedge sda) if(scl===1'b1) begin state=1;bits=0;incoming=0;low=0;end
 always @(posedge sda) if(scl===1'b1) begin state=0;low=0;end
 always @(posedge scl) begin
   if(state==1 || state==2 || state==3) begin
     incoming={incoming[6:0],sda};bits=bits+1;
     if(bits==8) begin
       bits=0;ack=1;ack_clocked=0;
       case(state)
         1: begin ack=incoming[7:1]==ADDRESS;next_state=incoming[0]?5:2;if(!ack)next_state=0;end
         2: begin pointer=ADS?incoming*2:incoming;next_state=3;end
         3: begin memory[pointer&255]=incoming;pointer=(pointer+1)&255;next_state=3;end
       endcase
       state=4;
     end
   end else if(state==4) ack_clocked=1;
   else if(state==5) begin bits=bits+1;if(bits==8)begin state=6;ack_clocked=0;end end
   else if(state==6) begin master_ack=!sda;ack_clocked=1;end
 end
 always @(negedge scl) begin
   if(state==4) begin
     if(!ack_clocked)low=ack;
     else begin
       low=0;state=next_state;bits=0;incoming=0;
       if(state==5)low=!memory[pointer&255][7];
     end
   end else if(state==5)low=!memory[pointer&255][7-bits];
   else if(state==6) begin
     low=0;
     if(ack_clocked)begin
       if(master_ack)begin pointer=(pointer+1)&255;bits=0;state=5;low=!memory[pointer][7];end
       else state=0;
     end
   end else low=0;
 end
endmodule
