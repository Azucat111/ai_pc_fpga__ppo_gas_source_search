# Run after build.tcl completed. Offline checkpoint inspection only.
set root [file normalize [file join [file dirname [info script]] ..]]
set run [file join $root build vivado car_overlay.runs impl_1]
open_checkpoint [file join $run car_wrapper_routed.dcp]
set out [file join $root build output]
report_utilization -hierarchical -file [file join $out utilization_hierarchical.rpt]
report_clocks -file [file join $out clocks.rpt]
set cells [get_cells -hier -filter {NAME =~ *tgs_pipeline}]
if {[llength $cells]!=1} {error "TGS hierarchy missing or ambiguous: $cells"}
report_utilization -cells $cells -file [file join $out tgs_utilization.rpt]
# Tool estimate only: no measured board power or energy claim.
report_power -file [file join $out estimated_power.rpt]
