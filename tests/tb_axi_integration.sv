`timescale 1ns/1ps
module tb_axi_integration;
 reg clk=0;always #100 clk=~clk;
 reg rstn=0;
 reg [11:0] awaddr=0,araddr=0;reg[31:0] wdata=0;
 reg [3:0] wstrb=15;reg awvalid=0,wvalid=0,bready=0,arvalid=0,rready=0;
 wire awready,wready,bvalid,arready,rvalid;wire[1:0] bresp,rresp;wire[31:0] rdata;
 reg a=0,b=0,serial_in=1;
 wire in1,in2,in3,in4,serial_out;
 tri1 scl,sda;
 car_axi dut(clk,rstn,awaddr,3'd0,awvalid,awready,wdata,wstrb,wvalid,wready,
 bresp,bvalid,bready,araddr,3'd0,arvalid,arready,rdata,rresp,rvalid,rready,
 a,b,1'b0,1'b0,serial_in,in1,in2,in3,in4,serial_out,,scl,sda);
 wire[7:0] telemetry_byte;wire telemetry_valid;wire[31:0] serial_errors;
 uart_rx #(.CLK_HZ(5000000)) monitor(clk,rstn,serial_out,telemetry_byte,telemetry_valid,serial_errors);
 integer saved=0,fd;
 integer brake_a=0,brake_b=0;
 always @(posedge clk)begin
   // A4950 both-high is an intentional, bounded wake/brake state.
   if(!rstn)begin brake_a=0;brake_b=0;end
   else begin
     brake_a=(in1 && in2)?brake_a+1:0;
     brake_b=(in3 && in4)?brake_b+1:0;
     if(brake_a>1250 || brake_b>1250)$fatal(1,"wake brake exceeded 250 us");
     if(!dut.core.drive && (in1 || in2 || in3 || in4))$fatal(1,"output while disabled/STOP");
   end
   if(telemetry_valid && saved<192)begin $fdisplay(fd,"%02x",telemetry_byte);saved<=saved+1;end
 end
 task aw(input[11:0] addr);begin @(negedge clk);awaddr=addr;awvalid=1;
 wait(awready);@(negedge clk);awvalid=0;end endtask
 task wd(input[31:0] value,input[3:0] strobe);begin @(negedge clk);wdata=value;wstrb=strobe;wvalid=1;
 wait(wready);@(negedge clk);wvalid=0;end endtask
 task write(input[11:0] addr,input[31:0] value,input[3:0] strobe,input order);
 begin
   if(order)begin wd(value,strobe);repeat(3)@(negedge clk);aw(addr);end
   else begin aw(addr);repeat(3)@(negedge clk);wd(value,strobe);end
   wait(bvalid);repeat(3)begin @(negedge clk);if(!bvalid || bresp)$fatal(1,"write response/backpressure");end
   bready=1;@(negedge clk);bready=0;
 end endtask
 task read(input[11:0] addr,input[31:0] expected);
 begin
   @(negedge clk);araddr=addr;arvalid=1;wait(arready);@(negedge clk);arvalid=0;
   wait(rvalid);repeat(4)begin @(negedge clk);if(!rvalid || rdata!==expected || rresp)$fatal(1,"read/backpressure %h got %h expected %h",addr,rdata,expected);end
   rready=1;@(negedge clk);rready=0;
 end endtask
 task serial_byte(input[7:0] data);
 integer bit_no;begin
   serial_in=0;#2170.138889;
   for(bit_no=0;bit_no<8;bit_no=bit_no+1)begin serial_in=data[bit_no];#2170.138889;end
   serial_in=1;#2170.138889;
 end endtask
 task encoder_phase(input[1:0] phase);begin {a,b}=phase;repeat(50)@(negedge clk);end endtask
 reg[7:0] vector[0:31];integer i,saw_pwm=0;
 initial begin
   $readmemh("build/tests/integration_command.hex",vector);
   fd=$fopen("build/tests/integration_telemetry.hex","w");
   repeat(4)@(negedge clk);rstn=1;
   read(0,32'h43415231);
   read(12'h064,32'h44424731);read(12'h068,5000000);
   read(12'h0c8,250);write(12'h0c8,0,15,0);read(12'h0c8,250);
   read(12'h0cc,32'h494d5532);write(12'h0cc,0,15,0);read(12'h0cc,32'h494d5532);
   write(12'h070,32'hdeadbeef,15,0);read(12'h070,0);
   write(12'h01c,32'haabbccdd,15,0);
   write(12'h01c,32'h00112233,4'b0100,1);
   read(12'h01c,32'haa11ccdd);write(12'h01c,2560,15,0);
   write(12'h010,32'h12345678,15,1);write(12'h014,1,15,0);
   // Runtime configuration is locked while enabled.
   write(12'h01c,999,15,1);read(12'h01c,2560);
   for(i=0;i<32;i=i+1)serial_byte(vector[i]);
   encoder_phase(1);encoder_phase(3);encoder_phase(2);encoder_phase(0);
   while(dut.core.now_ms<40)begin @(negedge clk);if(in1 || in2 || in3 || in4)saw_pwm=1;end
   if(!saw_pwm || dut.core.mode!=1 || dut.core.left_count!=4)$fatal(1,"UART -> motion/encoder/PWM integration");
   read(12'h0a0,1);read(12'h0a4,131072);read(12'h0a8,131072);
   wait(dut.core.now_ms>=120);repeat(3)@(negedge clk);
   if(dut.core.mode!=0 || dut.core.stop_reason!=2 || in1 || in2 || in3 || in4)$fatal(1,"hardware lease failed with PS idle");
   read(12'h078,0);read(12'h07c,0);
   if(saved!=192 || serial_errors)$fatal(1,"telemetry UART integration");
   $fclose(fd);$display("PASS AXI + full core: AW/W ordering, WSTRB, backpressure, UART motion, encoder, PWM, PS-independent timeout");$finish;
 end
 initial begin #150000000;$fatal(1,"test timeout");end
endmodule
