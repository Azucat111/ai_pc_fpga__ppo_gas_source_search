# Run from any directory: vivado -mode batch -source <repo>/vivado/build.tcl
# Optional -tclargs project_only creates the project without implementation.
set root [file normalize [file join [file dirname [info script]] ..]]
if {![string match "2022.2*" [version -short]]} {
  puts "WARNING: designed for Vivado 2022.2; current version is [version -short]"
}
set build [file join $root build vivado]
foreach arg $argv {
  if {[string match "build_dir=*" $arg]} {
    set build [file normalize [file join $root [string range $arg 10 end]]]
  }
}
file mkdir $build
create_project car_overlay $build -part xc7z020clg400-1 -force
set_property target_language Verilog [current_project]
set_property simulator_language Mixed [current_project]
add_files [glob [file join $root rtl *.sv]]
# IPI 2022.2 rejects SystemVerilog module-reference tops. car_axi uses only
# Verilog-2001 syntax; its implementation children remain SystemVerilog.
set_property FILE_TYPE Verilog [get_files [file join $root rtl car_axi.sv]]
add_files -fileset constrs_1 [file join $root vivado car.xdc]
update_compile_order -fileset sources_1
create_bd_design car
set ps7 [create_bd_cell -type ip -vlnv xilinx.com:ip:processing_system7:5.5 ps7]
# Reuse only the pinned vendor PS configuration block, not the vendor project script.
set f [open [file join $root vendor pynq pynqz2.tcl] r]
set preset [read $f];close $f
if {![regexp {set_property -dict \[ list .*?\] \$ps7} $preset config_command]} {
  error "Could not locate pinned PYNQ-Z2 PS configuration"
}
eval $config_command
set_property -dict [list CONFIG.PCW_USE_M_AXI_GP0 {1} \
  CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ {5.000000} \
  CONFIG.PCW_EN_CLK0_PORT {1} CONFIG.PCW_EN_RST0_PORT {1} \
  CONFIG.PCW_EN_CLK1_PORT {0} CONFIG.PCW_EN_CLK2_PORT {0} CONFIG.PCW_EN_CLK3_PORT {0}] $ps7
create_bd_intf_port -mode Master -vlnv xilinx.com:interface:ddrx_rtl:1.0 DDR
create_bd_intf_port -mode Master -vlnv xilinx.com:display_processing_system7:fixedio_rtl:1.0 FIXED_IO
connect_bd_intf_net [get_bd_intf_ports DDR] [get_bd_intf_pins ps7/DDR]
connect_bd_intf_net [get_bd_intf_ports FIXED_IO] [get_bd_intf_pins ps7/FIXED_IO]
create_bd_cell -type module -reference car_axi car_control
create_bd_cell -type ip -vlnv xilinx.com:ip:axi_interconnect:2.1 axi_bus
set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {1}] [get_bd_cells axi_bus]
create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 reset_controller
# Reset polarity is propagated from PS7 FCLK_RESET0_N in Vivado 2022.2.
# C_EXT_RESET_HIGH is read-only in the block design.
create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant:1.1 locked
set_property CONFIG.CONST_VAL 1 [get_bd_cells locked]
connect_bd_net [get_bd_pins locked/dout] [get_bd_pins reset_controller/dcm_locked]
connect_bd_net [get_bd_pins ps7/FCLK_RESET0_N] [get_bd_pins reset_controller/ext_reset_in]
connect_bd_net [get_bd_pins ps7/FCLK_CLK0] [get_bd_pins ps7/M_AXI_GP0_ACLK] \
  [get_bd_pins car_control/s_axi_aclk] [get_bd_pins axi_bus/ACLK] \
  [get_bd_pins axi_bus/S00_ACLK] [get_bd_pins axi_bus/M00_ACLK] [get_bd_pins reset_controller/slowest_sync_clk]
connect_bd_net [get_bd_pins reset_controller/interconnect_aresetn] [get_bd_pins axi_bus/ARESETN]
connect_bd_net [get_bd_pins reset_controller/peripheral_aresetn] [get_bd_pins car_control/s_axi_aresetn] \
  [get_bd_pins axi_bus/S00_ARESETN] [get_bd_pins axi_bus/M00_ARESETN]
connect_bd_intf_net [get_bd_intf_pins ps7/M_AXI_GP0] [get_bd_intf_pins axi_bus/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins axi_bus/M00_AXI] [get_bd_intf_pins car_control/S_AXI]
foreach pin {enc_m1_a enc_m1_b enc_m2_a enc_m2_b esp_rx} {
  create_bd_port -dir I $pin
  connect_bd_net [get_bd_ports $pin] [get_bd_pins car_control/$pin]
}
foreach pin {motor_a_in1 motor_a_in2 motor_b_in1 motor_b_in2 esp_tx source_led} {
  create_bd_port -dir O $pin
  connect_bd_net [get_bd_ports $pin] [get_bd_pins car_control/$pin]
}
foreach pin {i2c_scl i2c_sda} {
  create_bd_port -dir IO $pin
  connect_bd_net [get_bd_ports $pin] [get_bd_pins car_control/$pin]
}
set segments [get_bd_addr_segs -of_objects [get_bd_intf_pins car_control/S_AXI]]
if {[llength $segments]!=1} {error "Expected one car_control AXI register segment: $segments"}
assign_bd_address -offset 0x43C00000 -range 4K -target_address_space [get_bd_addr_spaces ps7/Data] $segments
validate_bd_design
if {[get_property CONFIG.C_EXT_RESET_HIGH [get_bd_cells reset_controller]] != 0} {
  error "PS7 reset must propagate as active-low"
}
save_bd_design
set bdfile [get_files */car.bd]
# Compile the BD with the top level so inferred I2C open-drain buffers reach
# physical pads. Per-IP OOC synthesis converts these internal tristates to LUTs.
set_property synth_checkpoint_mode None $bdfile
generate_target all $bdfile
set wrapper [make_wrapper -files $bdfile -top]
add_files -norecurse $wrapper
set_property top car_wrapper [current_fileset]
update_compile_order -fileset sources_1
if {[lsearch -exact $argv project_only]>=0} {
  puts "Created [file join $build car_overlay.xpr]"
  return
}
launch_runs synth_1 -jobs 4
wait_on_run synth_1
if {[get_property PROGRESS [get_runs synth_1]] ne "100%"} {error "Synthesis failed"}
launch_runs impl_1 -to_step write_bitstream -jobs 4
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] ne "100%"} {error "Implementation failed"}
open_run impl_1
set out [file join $root build output]
file mkdir $out
report_timing_summary -file [file join $out timing_summary.rpt]
report_utilization -file [file join $out utilization.rpt]
report_drc -file [file join $out drc.rpt]
report_io -file [file join $out io.rpt]
set worst [get_timing_paths -delay_type max -max_paths 1]
set worst_hold [get_timing_paths -delay_type min -max_paths 1]
if {[llength $worst]==0 || [llength $worst_hold]==0 || [get_property SLACK $worst]<0 || [get_property SLACK $worst_hold]<0} {
  error "Setup/hold timing closure required; see timing_summary.rpt"
}
file copy -force [file join $build car_overlay.runs impl_1 car_wrapper.bit] [file join $out car.bit]
file copy -force [file join $build car_overlay.gen sources_1 bd car hw_handoff car.hwh] [file join $out car.hwh]
set export_dir [file join $root build hardware_export]
file mkdir $export_dir
write_hw_platform -fixed -include_bit -force -file [file join $export_dir car.xsa]
puts "Outputs: $out"
