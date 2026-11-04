# Extension schematic 2026-10-18(1); Arduino header physical pin mapping.
set_property -dict {PACKAGE_PIN U13 IOSTANDARD LVCMOS33} [get_ports enc_m1_a]
set_property -dict {PACKAGE_PIN U17 IOSTANDARD LVCMOS33} [get_ports enc_m1_b]
set_property -dict {PACKAGE_PIN P18 IOSTANDARD LVCMOS33} [get_ports enc_m2_a]
set_property -dict {PACKAGE_PIN N17 IOSTANDARD LVCMOS33} [get_ports enc_m2_b]
set_property -dict {PACKAGE_PIN V13 IOSTANDARD LVCMOS33} [get_ports motor_a_in1]
set_property -dict {PACKAGE_PIN T15 IOSTANDARD LVCMOS33} [get_ports motor_a_in2]
set_property -dict {PACKAGE_PIN R16 IOSTANDARD LVCMOS33} [get_ports motor_b_in1]
set_property -dict {PACKAGE_PIN V18 IOSTANDARD LVCMOS33} [get_ports motor_b_in2]
set_property -dict {PACKAGE_PIN T14 IOSTANDARD LVCMOS33} [get_ports esp_rx]
set_property -dict {PACKAGE_PIN U12 IOSTANDARD LVCMOS33} [get_ports esp_tx]
set_property -dict {PACKAGE_PIN P16 IOSTANDARD LVCMOS33} [get_ports i2c_sda]
set_property -dict {PACKAGE_PIN P15 IOSTANDARD LVCMOS33} [get_ports i2c_scl]
set_property SLEW SLOW [get_ports {motor_* esp_tx i2c_*}]
set_property DRIVE 8 [get_ports {motor_* esp_tx i2c_*}]
# Asynchronous peripherals: first-stage synchronizers are marked ASYNC_REG in RTL.
set_false_path -from [get_ports {enc_* esp_rx i2c_*}]
set_false_path -to [get_ports {motor_* esp_tx i2c_*}]

# PYNQ v3.0.1 official base.xdc: LD5 BLUE, PL output, active-high transistor drive.
set_property -dict {PACKAGE_PIN G14 IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 8} [get_ports source_led]
