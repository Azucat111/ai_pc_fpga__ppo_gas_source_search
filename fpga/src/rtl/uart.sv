`timescale 1ns/1ps
module uart_rx #(parameter integer CLK_HZ=50000000,BAUD=460800)(
 input wire clk,rstn,rx,output reg [7:0] data,output reg valid,
 output reg [31:0] framing_errors);
 localparam integer DIV=(CLK_HZ+BAUD/2)/BAUD;
 (* ASYNC_REG="TRUE" *) reg [1:0] sync;
 integer timer,bit_index;
 reg [1:0] state;
 reg [7:0] shift;
 always @(posedge clk) begin
   if (!rstn) begin sync<=3;state<=0;timer<=0;bit_index<=0;shift<=0;data<=0;valid<=0;framing_errors<=0;end
   else begin
     sync<={sync[0],rx};valid<=0;
     case(state)
       0: if (!sync[1]) begin state<=1;timer<=DIV/2;end
       1: if(timer!=0) timer<=timer-1;
          else if(!sync[1]) begin state<=2;timer<=DIV-1;bit_index<=0;end
          else state<=0;
       2: if(timer!=0) timer<=timer-1;
          else begin shift[bit_index]<=sync[1];timer<=DIV-1;
            if(bit_index==7) state<=3;else bit_index<=bit_index+1;end
       3: if(timer!=0) timer<=timer-1;
          else begin state<=0;
            if(sync[1]) begin data<=shift;valid<=1;end
            else framing_errors<=framing_errors+1;end
     endcase
   end
 end
endmodule

module uart_tx #(parameter integer CLK_HZ=50000000,BAUD=460800)(
 input wire clk,rstn,valid,input wire [7:0] data,
 output wire ready,output wire tx);
 localparam integer DIV=(CLK_HZ+BAUD/2)/BAUD;
 reg [9:0] shift;
 integer timer,remaining;
 assign ready=remaining==0;
 assign tx=remaining==0 ? 1'b1:shift[0];
 always @(posedge clk) begin
   if(!rstn) begin shift<=10'h3ff;timer<=0;remaining<=0;end
   else if(remaining==0) begin
     if(valid) begin shift<={1'b1,data,1'b0};remaining<=10;timer<=DIV-1;end
   end else if(timer==0) begin
     shift<={1'b1,shift[9:1]};remaining<=remaining-1;timer<=DIV-1;
   end else timer<=timer-1;
 end
endmodule
