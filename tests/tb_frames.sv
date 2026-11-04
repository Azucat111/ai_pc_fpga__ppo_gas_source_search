`timescale 1ns/1ps
module tb_frames;
 reg clk=0;always #5 clk=~clk;
 reg rstn=0,ms_tick=0,bvalid=0;reg[7:0] bdata=0;
 wire command_valid;wire[255:0] command;wire[31:0] errors;
 frame_rx rx(clk,rstn,ms_tick,bvalid,bdata,command_valid,command,errors);
 reg[7:0] vector[0:31];integer valid_count=0;
 always @(posedge clk)if(command_valid)valid_count<=valid_count+1;
 reg start=0,ready=0;reg[1439:0] payload=0;
 wire out_valid,busy;wire[7:0] out_data;
 frame_tx tx(clk,rstn,start,payload,32'h12345678,ready,out_valid,out_data,busy);
 integer out_file,index=0,cycle=0;
 always @(negedge clk)begin cycle=cycle+1;ready=(cycle%3)!=0;end
 always @(posedge clk)if(out_valid && ready)begin $fdisplay(out_file,"%02x",out_data);index<=index+1;end
 task byte_in(input[7:0] v);begin @(negedge clk);bdata=v;bvalid=1;@(negedge clk);bvalid=0;end endtask
 integer i;
 initial begin
   $readmemh("build/tests/command.hex",vector);
   out_file=$fopen("build/tests/telemetry.hex","w");
   repeat(3)@(negedge clk);rstn=1;
   byte_in(8'h11);byte_in(8'ha5);byte_in(8'ha5);
   for(i=1;i<32;i=i+1)byte_in(vector[i]);
   repeat(3)@(negedge clk);
   if(valid_count!=1 || command[191:160]!=32'hfffe0000)$fatal(1,"frame parsing");
   for(i=0;i<32;i=i+1)byte_in(i==22?vector[i]^1:vector[i]);
   if(errors!=1 || valid_count!=1)$fatal(1,"CRC rejection");
   for(i=0;i<8;i=i+1)byte_in(vector[i]);
   repeat(6)begin @(negedge clk);ms_tick=1;@(negedge clk);ms_tick=0;end
   for(i=0;i<32;i=i+1)byte_in(vector[i]);
   repeat(3)@(negedge clk);if(valid_count!=2)$fatal(1,"frame timeout recovery");
   for(i=0;i<45;i=i+1)payload[i*32+:32]=i;
   @(negedge clk);start=1;@(negedge clk);start=0;
   wait(!busy);repeat(2)@(negedge clk);$fclose(out_file);
   if(index!=192)$fatal(1,"telemetry length %d",index);
   $display("PASS framing: sync, CRC rejection, timeout, backpressure, telemetry serialization");$finish;
 end
 initial begin #100000;$fatal(1,"test timeout");end
endmodule
