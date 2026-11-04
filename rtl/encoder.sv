`timescale 1ns/1ps
module encoder #(parameter integer FILTER_CYCLES=50)(
 input wire clk, rstn, a, b, invert,
 output reg signed [63:0] count,
 output reg [31:0] invalid_count
);
 (* ASYNC_REG="TRUE" *) reg [1:0] sync1, sync2;
 reg [1:0] candidate, stable, previous;
 reg initialized;
 integer filter_count;
 reg signed [2:0] step;
 always @* begin
   step=0;
   case ({previous,stable})
     4'b0001,4'b0111,4'b1110,4'b1000: step=1;
     4'b0010,4'b1011,4'b1101,4'b0100: step=-1;
     default: step=0;
   endcase
 end
 always @(posedge clk) begin
   if (!rstn) begin
     sync1<=0;sync2<=0;candidate<=0;stable<=0;previous<=0;
     filter_count<=0;count<=0;invalid_count<=0;initialized<=0;
   end else begin
     sync1<={a,b};sync2<=sync1;
     if (sync2!=candidate) begin candidate<=sync2;filter_count<=0;end
     else if (filter_count<FILTER_CYCLES) filter_count<=filter_count+1;
     previous<=stable;
     if (sync2==candidate && filter_count>=FILTER_CYCLES) begin
       stable<=candidate;
       if(!initialized) begin previous<=candidate;initialized<=1;end
     end
     if (initialized && previous!=stable) begin
       if ((previous^stable)==2'b11) invalid_count<=invalid_count+1;
       else if (invert) count<=count-step;
       else count<=count+step;
     end
   end
 end
endmodule
