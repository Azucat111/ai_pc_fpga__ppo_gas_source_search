`timescale 1ns/1ps
// One independent fixed-point lane. Updates ONLY on a new valid conversion.
// Signed ADC code domain; no voltage/concentration calibration is implied.
module tgs_channel #(parameter integer ON=200, OFF=100, HITS=3)(
 input wire clk,rstn,sample_valid,input wire signed [15:0] sample,
 input wire [31:0] sample_time,
 output reg updated,output reg signed [31:0] filtered_q8,baseline_q8,delta_code,slope_code,
 output reg [31:0] stamp,count,peak,events,output reg active);
 reg initialized; reg [7:0] hits;
 reg signed [31:0] x,fnext,bnext,dnext,snext;
 reg [31:0] magnitude;
 always @* begin
  x=$signed(sample)*32'sd256;
  fnext=initialized ? filtered_q8+((x-filtered_q8)>>>2):x;
  // Freeze baseline during candidates and events to avoid immediate adaptation.
  bnext=initialized ? baseline_q8:x;
  dnext=(fnext-bnext)>>>8;
  magnitude=dnext<0 ? -dnext:dnext;
  if(initialized && !active && hits==0 && magnitude<OFF)
    bnext=baseline_q8+((fnext-baseline_q8)>>>6);
  dnext=(fnext-bnext)>>>8;
  magnitude=dnext<0 ? -dnext:dnext;
  snext=initialized ? (fnext-filtered_q8)>>>8:0;
 end
 always @(posedge clk) begin
  if(!rstn) begin
   updated<=0;initialized<=0;hits<=0;filtered_q8<=0;baseline_q8<=0;
   delta_code<=0;slope_code<=0;stamp<=0;count<=0;peak<=0;events<=0;active<=0;
  end else begin
   updated<=0;
   if(sample_valid) begin
    initialized<=1;updated<=1;filtered_q8<=fnext;baseline_q8<=bnext;
    delta_code<=dnext;slope_code<=snext;stamp<=sample_time;count<=count+1;
    if(magnitude>peak)peak<=magnitude;
    if(!active)begin
     if(magnitude>=ON)begin
      if(hits>=HITS-1)begin active<=1;hits<=0;events<=events+1;end
      else hits<=hits+1;
     end else hits<=0;
    end else if(magnitude<=OFF)begin active<=0;hits<=0;end
   end
  end
 end
endmodule
