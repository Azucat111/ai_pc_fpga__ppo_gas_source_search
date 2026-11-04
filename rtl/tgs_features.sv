`timescale 1ns/1ps
// Parallel read-only sensor consumer. No motion outputs or bus requests.
// 16 x 512-bit FIFO, drop newest on full, stable head until explicit POP.
module tgs_features(input wire clk,rstn,input wire [31:0] now_ms,
 input wire signed [15:0] adc0,adc1,input wire [1:0] valid,
 input wire [31:0] time0,time1,input wire wr_en,input wire [11:0] wr_addr,rd_addr,
 input wire [31:0] wr_data,input wire [3:0] wr_strb,output reg [31:0] rd_data);
 reg [1:0] sampled_valid; // Pipeline metadata with the sample epoch, not live status.
 reg [31:0] seen0,seen1; reg signed [15:0] raw0,raw1;
 wire new0=valid[0] && time0!=seen0, new1=valid[1] && time1!=seen1;
 wire u0,u1,a0,a1;
 wire signed [31:0] f0,f1,b0,b1,d0,d1,s0,s1;
 wire [31:0] t0,t1,c0,c1,p0,p1,e0,e1;
 tgs_channel lane0(clk,rstn,new0,adc0,time0,u0,f0,b0,d0,s0,t0,c0,p0,e0,a0);
 tgs_channel lane1(clk,rstn,new1,adc1,time1,u1,f1,b1,d1,s1,t1,c1,p1,e1,a1);
 reg [511:0] fifo[0:15];reg [3:0] wp,rp;reg [4:0] used;
 reg [31:0] overflow,records,window_start,n0,n1,peak0,peak1;
 wire clear_window=wr_en && wr_strb==15 && wr_addr==12'h524 && wr_data[0];
 wire pop=wr_en && wr_strb==15 && wr_addr==12'h520 && wr_data[0] && used!=0;
 wire push=(u0||u1) && used<16;
 wire [31:0] mag0=d0<0 ? -d0:d0,mag1=d1<0 ? -d1:d1;
 wire [511:0] record_data={s1,s0,d1,d0,b1,b0,f1,f0,t1,t0,{{16{raw1[15]}},raw1},{{16{raw0[15]}},raw0},now_ms,
                  {24'b0,sampled_valid,a1,a0,u1,u0,2'b0},records,32'h54475331};
 always @* begin
  rd_data=0;
  case(rd_addr)
   12'h500:rd_data=32'h54475331;
   12'h504:rd_data={27'b0,used};
   12'h508:rd_data=overflow;
   12'h50c:rd_data=records;
   12'h510:rd_data=200;
   12'h514:rd_data=100;
   12'h518:rd_data=3;
   12'h528:rd_data=window_start;
   12'h52c:rd_data=n0;
   12'h530:rd_data=n1;
   12'h534:rd_data=peak0;
   12'h538:rd_data=peak1;
   12'h53c:rd_data={28'b0,valid,a1,a0};
   12'h580:rd_data=e0;
   12'h584:rd_data=e1;
   12'h588:rd_data=p0;
   12'h58c:rd_data=p1;
   12'h590:rd_data=now_ms-t0;
   12'h594:rd_data=now_ms-t1;
   12'h598:rd_data=c0;12'h59c:rd_data=c1;
  endcase
  if(rd_addr>=12'h540 && rd_addr<12'h580 && used!=0)
   rd_data=fifo[rp][(rd_addr-12'h540)*8+:32];
 end
 always @(posedge clk)begin
  if(!rstn)begin
   sampled_valid<=0;raw0<=0;raw1<=0;seen0<=32'hffffffff;seen1<=32'hffffffff;wp<=0;rp<=0;used<=0;
   overflow<=0;records<=0;window_start<=0;n0<=0;n1<=0;peak0<=0;peak1<=0;
  end else begin
   sampled_valid<=valid;
   if(new0)begin seen0<=time0;raw0<=adc0;end
   if(new1)begin seen1<=time1;raw1<=adc1;end
   if(u0||u1)begin
    records<=records+1;
    if(push)begin fifo[wp]<=record_data;wp<=wp+1;end
    else overflow<=overflow+1;
   end
   if(pop)rp<=rp+1;
   case({push,pop})2'b10:used<=used+1;2'b01:used<=used-1;default:;endcase
   if(clear_window)begin
    window_start<=now_ms;n0<=0;n1<=0;peak0<=0;peak1<=0;
   end else begin
    if(u0)begin n0<=n0+1;if(mag0>peak0)peak0<=mag0;end
    if(u1)begin n1<=n1+1;if(mag1>peak1)peak1<=mag1;end
   end
  end
 end
endmodule
