`timescale 1ns/1ps
// Independent application state. No enable, action or motor outputs.
module source_indicator(input wire clk,rstn,ms_tick,
 input wire [31:0] boot_id,now_ms,input wire request_valid,
 input wire [255:0] request,input wire tx_slot,byte_ready,
 output wire byte_valid,output wire [7:0] byte_data,
 output wire busy,starting,output reg confirmed,output reg led,
 output reg [31:0] last_sequence);
 reg [31:0] session,ack_sequence;
 reg seen,pending;reg [239:0] last_request;
 reg [8:0] blink_ms;
 reg [1439:0] reply;
 reg [31:0] status;reg next_confirmed;
 wire [31:0] seq=request[63:32],req_boot=request[95:64],issued=request[127:96];
 wire [7:0] event_code=request[135:128];
 wire [15:0] ttl=request[159:144];
 assign starting=pending && tx_slot && !busy;
 frame_tx #(.FRAME_TYPE(8'h83)) serializer(clk,rstn,starting,reply,ack_sequence,
   byte_ready,byte_valid,byte_data,busy);
 always @* begin
   status=0;next_confirmed=confirmed;
   if(req_boot!=boot_id || boot_id==0)status=1;
   else if(request[143:136]!=0 || request[239:160]!=0 || event_code>2)status=4;
   else if(event_code==2)begin
     if(ttl<100 || ttl>500 || now_ms-issued>ttl)status=3;
   end else if(seen && seq==last_sequence)begin
     if(request[239:0]!=last_request)status=5;
   end else if(seen && $signed(seq-last_sequence)<=0)status=2;
   else if(ttl<100 || ttl>500 || now_ms-issued>ttl)status=3;
   else if(event_code!=2)next_confirmed=(event_code==1);
 end
 always @(posedge clk)begin
   if(!rstn)begin
     session<=0;ack_sequence<=0;seen<=0;pending<=0;last_request<=0;
     blink_ms<=0;reply<=0;confirmed<=0;led<=0;last_sequence<=0;
   end else if(session!=boot_id)begin
     session<=boot_id;seen<=0;pending<=0;last_request<=0;last_sequence<=0;
     blink_ms<=0;confirmed<=0;led<=0;
   end else begin
     if(starting)pending<=0;
     if(ms_tick && confirmed)begin
       if(blink_ms==249)begin blink_ms<=0;led<=!led;end
       else blink_ms<=blink_ms+1;
     end
     if(request_valid)begin
       reply<=0;pending<=1;ack_sequence<=ack_sequence+1;
       reply[0*32+:32]<=boot_id;reply[1*32+:32]<=now_ms;
       reply[2*32+:32]<=seq;reply[3*32+:32]<=status;
       reply[4*32+:32]<={24'b0,event_code};
       reply[5*32+:32]<={31'b0,next_confirmed};
       reply[6*32+:32]<=last_sequence;reply[7*32+:32]<=250;
       reply[8*32+:32]<=32'h4c454431; // LED1 capability.
       if(status==0 && event_code!=2 && (!seen || seq!=last_sequence))begin
         seen<=1;last_sequence<=seq;last_request<=request[239:0];
         reply[6*32+:32]<=seq;
         confirmed<=next_confirmed;
         // Repeated confirmation, including new seq, must not restart blink phase.
         if(next_confirmed!=confirmed)begin blink_ms<=0;led<=next_confirmed;end
       end
     end
   end
 end
endmodule
