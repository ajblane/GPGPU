# Copyright (C) 1991-2014 Altera Corporation. All rights reserved.
# Your use of Altera Corporation's design tools, logic functions 
# and other software and tools, and its AMPP partner logic 
# functions, and any output files from any of the foregoing 
# (including device programming or simulation files), and any 
# associated documentation or information are expressly subject 
# to the terms and conditions of the Altera Program License 
# Subscription Agreement, the Altera Quartus II License Agreement,
# the Altera MegaCore Function License Agreement, or other 
# applicable license agreement, including, without limitation, 
# that your use is for the sole purpose of programming logic 
# devices manufactured by Altera and sold by Altera or its 
# authorized distributors.  Please refer to the applicable 
# agreement for further details.

# Quartus II: Generate Tcl File for Project
# File: helio_ghrd_top.tcl
# Generated on: Sun Sep 28 17:06:20 2014

# Load Quartus II Tcl Project package
package require ::quartus::project

set need_to_close_project 0
set make_assignments 1

# Check that the right project is open
if {[is_project_open]} {
	if {[string compare $quartus(project) "helio_ghrd_top"]} {
		puts "Project helio_ghrd_top is not open"
		set make_assignments 0
	}
} else {
	# Only open if not already open
	if {[project_exists helio_ghrd_top]} {
		project_open -revision helio_ghrd_top helio_ghrd_top
	} else {
		project_new -revision helio_ghrd_top helio_ghrd_top
	}
	set need_to_close_project 1
}

# Make assignments
if {$make_assignments} {
	set_global_assignment -name ORIGINAL_QUARTUS_VERSION 14.0
	set_global_assignment -name PROJECT_CREATION_TIME_DATE "10:56:29  JULY 22, 2014"
	set_global_assignment -name LAST_QUARTUS_VERSION 14.0
	set_global_assignment -name PROJECT_OUTPUT_DIRECTORY output_files
	set_global_assignment -name QIP_FILE ip/altsource_probe/hps_reset.qip
	set_global_assignment -name VERILOG_FILE ip/intr_capturer/intr_capturer.v
	set_global_assignment -name VERILOG_FILE ip/edge_detect/altera_edge_detector.v
	set_global_assignment -name VERILOG_FILE ip/debounce/debounce.v
	set_global_assignment -name QIP_FILE soc_system/synthesis/soc_system.qip
	set_global_assignment -name SDC_FILE soc_system_timing.sdc
	set_global_assignment -name VERILOG_FILE helio_ghrd_top.v
	set_global_assignment -name MIN_CORE_JUNCTION_TEMP 0
	set_global_assignment -name MAX_CORE_JUNCTION_TEMP 85
	set_global_assignment -name USE_DLL_FREQUENCY_FOR_DQS_DELAY_CHAIN ON
	set_global_assignment -name FAMILY "Cyclone V"
	set_global_assignment -name DEVICE 5CSXFC6C6U23C8ES
	set_global_assignment -name STRATIX_DEVICE_IO_STANDARD "2.5 V"
	set_global_assignment -name UNIPHY_SEQUENCER_DQS_CONFIG_ENABLE ON
	set_global_assignment -name OPTIMIZE_MULTI_CORNER_TIMING ON
	set_global_assignment -name ECO_REGENERATE_REPORT ON
	set_global_assignment -name ENABLE_CONFIGURATION_PINS OFF
	set_global_assignment -name ENABLE_BOOT_SEL_PIN OFF
	set_global_assignment -name STRATIXV_CONFIGURATION_SCHEME "ACTIVE SERIAL X4"
	set_global_assignment -name CRC_ERROR_OPEN_DRAIN ON
	set_global_assignment -name ACTIVE_SERIAL_CLOCK FREQ_100MHZ
	set_global_assignment -name USE_CONFIGURATION_DEVICE OFF
	set_global_assignment -name STRATIXII_CONFIGURATION_DEVICE EPCQ256
	set_global_assignment -name POWER_PRESET_COOLING_SOLUTION "23 MM HEAT SINK WITH 200 LFPM AIRFLOW"
	set_global_assignment -name POWER_BOARD_THERMAL_MODEL "NONE (CONSERVATIVE)"
	set_global_assignment -name OUTPUT_IO_TIMING_NEAR_END_VMEAS "HALF VCCIO" -rise
	set_global_assignment -name OUTPUT_IO_TIMING_NEAR_END_VMEAS "HALF VCCIO" -fall
	set_global_assignment -name OUTPUT_IO_TIMING_FAR_END_VMEAS "HALF SIGNAL SWING" -rise
	set_global_assignment -name OUTPUT_IO_TIMING_FAR_END_VMEAS "HALF SIGNAL SWING" -fall
	set_global_assignment -name PARTITION_NETLIST_TYPE SOURCE -section_id Top
	set_global_assignment -name PARTITION_FITTER_PRESERVATION_LEVEL PLACEMENT_AND_ROUTING -section_id Top
	set_global_assignment -name PARTITION_COLOR 16764057 -section_id Top

	
	
	
	set_location_assignment PIN_Y13 -to fpga_clk_50
	set_location_assignment PIN_V12 -to fpga_clk_100
	set_location_assignment PIN_AA4 -to fpga_dipsw_pio[3]
	set_location_assignment PIN_W8 -to fpga_dipsw_pio[0]
	set_location_assignment PIN_AB4 -to fpga_dipsw_pio[1]
	set_location_assignment PIN_T8 -to fpga_dipsw_pio[2]
	set_location_assignment PIN_U9 -to fpga_led_pio[0]
	set_location_assignment PIN_AD4 -to fpga_led_pio[1]
	set_location_assignment PIN_V10 -to fpga_led_pio[2]
	set_location_assignment PIN_AC4 -to fpga_led_pio[3]
	set_location_assignment PIN_Y4 -to fpga_button_pio[0]
	set_location_assignment PIN_Y8 -to fpga_button_pio[1]
	set_location_assignment PIN_Y5 -to fpga_button_pio[2]
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_dq[0] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_dq[1] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_dq[2] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_dq[3] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_dq[4] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_dq[5] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_dq[6] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_dq[7] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_dq[8] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_dq[9] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_dq[10] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_dq[11] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_dq[12] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_dq[13] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_dq[14] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_dq[15] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_dq[16] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_dq[17] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_dq[18] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_dq[19] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_dq[20] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_dq[21] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_dq[22] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_dq[23] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_dq[24] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_dq[25] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_dq[26] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_dq[27] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_dq[28] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_dq[29] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_dq[30] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_dq[31] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_dm[0] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_dm[1] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_dm[2] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_dm[3] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_dqs[0] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_dqs[1] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_dqs[2] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_dqs[3] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_dqs_n[0] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_dqs_n[1] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_dqs_n[2] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_dqs_n[3] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_a[0] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_a[10] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_a[11] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_a[12] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_a[13] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_a[14] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_a[1] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_a[2] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_a[3] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_a[4] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_a[5] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_a[6] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_a[7] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_a[8] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_a[9] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_ba[0] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_ba[1] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_ba[2] -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_cas_n -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_cke -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_cs_n -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_odt -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_ras_n -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_we_n -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_reset_n -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_ck -tag __hps_sdram_p0
	set_instance_assignment -name PACKAGE_SKEW_COMPENSATION OFF -to hps_memory_mem_ck_n -tag __hps_sdram_p0
	set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to hps_trace_CLK
	set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to hps_trace_D0
	set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to hps_trace_D1
	set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to hps_trace_D2
	set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to hps_trace_D3
	set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to hps_trace_D4
	set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to hps_trace_D5
	set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to hps_trace_D6
	set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to hps_trace_D7
	set_instance_assignment -name SLEW_RATE 1 -to hps_trace_CLK
	set_instance_assignment -name SLEW_RATE 1 -to hps_trace_D0
	set_instance_assignment -name SLEW_RATE 1 -to hps_trace_D1
	set_instance_assignment -name SLEW_RATE 1 -to hps_trace_D2
	set_instance_assignment -name SLEW_RATE 1 -to hps_trace_D3
	set_instance_assignment -name SLEW_RATE 1 -to hps_trace_D4
	set_instance_assignment -name SLEW_RATE 1 -to hps_trace_D5
	set_instance_assignment -name SLEW_RATE 1 -to hps_trace_D6
	set_instance_assignment -name SLEW_RATE 1 -to hps_trace_D7
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_oct_rzqin -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_dq[0] -tag __hps_sdram_p0
	set_instance_assignment -name INPUT_TERMINATION "PARALLEL 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[0] -tag __hps_sdram_p0
	set_instance_assignment -name OUTPUT_TERMINATION "SERIES 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[0] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_dq[1] -tag __hps_sdram_p0
	set_instance_assignment -name INPUT_TERMINATION "PARALLEL 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[1] -tag __hps_sdram_p0
	set_instance_assignment -name OUTPUT_TERMINATION "SERIES 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[1] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_dq[2] -tag __hps_sdram_p0
	set_instance_assignment -name INPUT_TERMINATION "PARALLEL 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[2] -tag __hps_sdram_p0
	set_instance_assignment -name OUTPUT_TERMINATION "SERIES 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[2] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_dq[3] -tag __hps_sdram_p0
	set_instance_assignment -name INPUT_TERMINATION "PARALLEL 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[3] -tag __hps_sdram_p0
	set_instance_assignment -name OUTPUT_TERMINATION "SERIES 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[3] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_dq[4] -tag __hps_sdram_p0
	set_instance_assignment -name INPUT_TERMINATION "PARALLEL 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[4] -tag __hps_sdram_p0
	set_instance_assignment -name OUTPUT_TERMINATION "SERIES 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[4] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_dq[5] -tag __hps_sdram_p0
	set_instance_assignment -name INPUT_TERMINATION "PARALLEL 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[5] -tag __hps_sdram_p0
	set_instance_assignment -name OUTPUT_TERMINATION "SERIES 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[5] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_dq[6] -tag __hps_sdram_p0
	set_instance_assignment -name INPUT_TERMINATION "PARALLEL 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[6] -tag __hps_sdram_p0
	set_instance_assignment -name OUTPUT_TERMINATION "SERIES 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[6] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_dq[7] -tag __hps_sdram_p0
	set_instance_assignment -name INPUT_TERMINATION "PARALLEL 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[7] -tag __hps_sdram_p0
	set_instance_assignment -name OUTPUT_TERMINATION "SERIES 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[7] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_dq[8] -tag __hps_sdram_p0
	set_instance_assignment -name INPUT_TERMINATION "PARALLEL 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[8] -tag __hps_sdram_p0
	set_instance_assignment -name OUTPUT_TERMINATION "SERIES 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[8] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_dq[9] -tag __hps_sdram_p0
	set_instance_assignment -name INPUT_TERMINATION "PARALLEL 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[9] -tag __hps_sdram_p0
	set_instance_assignment -name OUTPUT_TERMINATION "SERIES 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[9] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_dq[10] -tag __hps_sdram_p0
	set_instance_assignment -name INPUT_TERMINATION "PARALLEL 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[10] -tag __hps_sdram_p0
	set_instance_assignment -name OUTPUT_TERMINATION "SERIES 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[10] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_dq[11] -tag __hps_sdram_p0
	set_instance_assignment -name INPUT_TERMINATION "PARALLEL 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[11] -tag __hps_sdram_p0
	set_instance_assignment -name OUTPUT_TERMINATION "SERIES 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[11] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_dq[12] -tag __hps_sdram_p0
	set_instance_assignment -name INPUT_TERMINATION "PARALLEL 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[12] -tag __hps_sdram_p0
	set_instance_assignment -name OUTPUT_TERMINATION "SERIES 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[12] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_dq[13] -tag __hps_sdram_p0
	set_instance_assignment -name INPUT_TERMINATION "PARALLEL 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[13] -tag __hps_sdram_p0
	set_instance_assignment -name OUTPUT_TERMINATION "SERIES 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[13] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_dq[14] -tag __hps_sdram_p0
	set_instance_assignment -name INPUT_TERMINATION "PARALLEL 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[14] -tag __hps_sdram_p0
	set_instance_assignment -name OUTPUT_TERMINATION "SERIES 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[14] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_dq[15] -tag __hps_sdram_p0
	set_instance_assignment -name INPUT_TERMINATION "PARALLEL 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[15] -tag __hps_sdram_p0
	set_instance_assignment -name OUTPUT_TERMINATION "SERIES 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[15] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_dq[16] -tag __hps_sdram_p0
	set_instance_assignment -name INPUT_TERMINATION "PARALLEL 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[16] -tag __hps_sdram_p0
	set_instance_assignment -name OUTPUT_TERMINATION "SERIES 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[16] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_dq[17] -tag __hps_sdram_p0
	set_instance_assignment -name INPUT_TERMINATION "PARALLEL 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[17] -tag __hps_sdram_p0
	set_instance_assignment -name OUTPUT_TERMINATION "SERIES 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[17] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_dq[18] -tag __hps_sdram_p0
	set_instance_assignment -name INPUT_TERMINATION "PARALLEL 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[18] -tag __hps_sdram_p0
	set_instance_assignment -name OUTPUT_TERMINATION "SERIES 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[18] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_dq[19] -tag __hps_sdram_p0
	set_instance_assignment -name INPUT_TERMINATION "PARALLEL 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[19] -tag __hps_sdram_p0
	set_instance_assignment -name OUTPUT_TERMINATION "SERIES 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[19] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_dq[20] -tag __hps_sdram_p0
	set_instance_assignment -name INPUT_TERMINATION "PARALLEL 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[20] -tag __hps_sdram_p0
	set_instance_assignment -name OUTPUT_TERMINATION "SERIES 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[20] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_dq[21] -tag __hps_sdram_p0
	set_instance_assignment -name INPUT_TERMINATION "PARALLEL 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[21] -tag __hps_sdram_p0
	set_instance_assignment -name OUTPUT_TERMINATION "SERIES 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[21] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_dq[22] -tag __hps_sdram_p0
	set_instance_assignment -name INPUT_TERMINATION "PARALLEL 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[22] -tag __hps_sdram_p0
	set_instance_assignment -name OUTPUT_TERMINATION "SERIES 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[22] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_dq[23] -tag __hps_sdram_p0
	set_instance_assignment -name INPUT_TERMINATION "PARALLEL 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[23] -tag __hps_sdram_p0
	set_instance_assignment -name OUTPUT_TERMINATION "SERIES 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[23] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_dq[24] -tag __hps_sdram_p0
	set_instance_assignment -name INPUT_TERMINATION "PARALLEL 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[24] -tag __hps_sdram_p0
	set_instance_assignment -name OUTPUT_TERMINATION "SERIES 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[24] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_dq[25] -tag __hps_sdram_p0
	set_instance_assignment -name INPUT_TERMINATION "PARALLEL 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[25] -tag __hps_sdram_p0
	set_instance_assignment -name OUTPUT_TERMINATION "SERIES 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[25] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_dq[26] -tag __hps_sdram_p0
	set_instance_assignment -name INPUT_TERMINATION "PARALLEL 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[26] -tag __hps_sdram_p0
	set_instance_assignment -name OUTPUT_TERMINATION "SERIES 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[26] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_dq[27] -tag __hps_sdram_p0
	set_instance_assignment -name INPUT_TERMINATION "PARALLEL 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[27] -tag __hps_sdram_p0
	set_instance_assignment -name OUTPUT_TERMINATION "SERIES 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[27] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_dq[28] -tag __hps_sdram_p0
	set_instance_assignment -name INPUT_TERMINATION "PARALLEL 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[28] -tag __hps_sdram_p0
	set_instance_assignment -name OUTPUT_TERMINATION "SERIES 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[28] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_dq[29] -tag __hps_sdram_p0
	set_instance_assignment -name INPUT_TERMINATION "PARALLEL 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[29] -tag __hps_sdram_p0
	set_instance_assignment -name OUTPUT_TERMINATION "SERIES 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[29] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_dq[30] -tag __hps_sdram_p0
	set_instance_assignment -name INPUT_TERMINATION "PARALLEL 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[30] -tag __hps_sdram_p0
	set_instance_assignment -name OUTPUT_TERMINATION "SERIES 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[30] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_dq[31] -tag __hps_sdram_p0
	set_instance_assignment -name INPUT_TERMINATION "PARALLEL 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[31] -tag __hps_sdram_p0
	set_instance_assignment -name OUTPUT_TERMINATION "SERIES 50 OHM WITH CALIBRATION" -to hps_memory_mem_dq[31] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "DIFFERENTIAL 1.5-V SSTL CLASS I" -to hps_memory_mem_dqs[0] -tag __hps_sdram_p0
	set_instance_assignment -name INPUT_TERMINATION "PARALLEL 50 OHM WITH CALIBRATION" -to hps_memory_mem_dqs[0] -tag __hps_sdram_p0
	set_instance_assignment -name OUTPUT_TERMINATION "SERIES 50 OHM WITH CALIBRATION" -to hps_memory_mem_dqs[0] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "DIFFERENTIAL 1.5-V SSTL CLASS I" -to hps_memory_mem_dqs[1] -tag __hps_sdram_p0
	set_instance_assignment -name INPUT_TERMINATION "PARALLEL 50 OHM WITH CALIBRATION" -to hps_memory_mem_dqs[1] -tag __hps_sdram_p0
	set_instance_assignment -name OUTPUT_TERMINATION "SERIES 50 OHM WITH CALIBRATION" -to hps_memory_mem_dqs[1] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "DIFFERENTIAL 1.5-V SSTL CLASS I" -to hps_memory_mem_dqs[2] -tag __hps_sdram_p0
	set_instance_assignment -name INPUT_TERMINATION "PARALLEL 50 OHM WITH CALIBRATION" -to hps_memory_mem_dqs[2] -tag __hps_sdram_p0
	set_instance_assignment -name OUTPUT_TERMINATION "SERIES 50 OHM WITH CALIBRATION" -to hps_memory_mem_dqs[2] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "DIFFERENTIAL 1.5-V SSTL CLASS I" -to hps_memory_mem_dqs[3] -tag __hps_sdram_p0
	set_instance_assignment -name INPUT_TERMINATION "PARALLEL 50 OHM WITH CALIBRATION" -to hps_memory_mem_dqs[3] -tag __hps_sdram_p0
	set_instance_assignment -name OUTPUT_TERMINATION "SERIES 50 OHM WITH CALIBRATION" -to hps_memory_mem_dqs[3] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "DIFFERENTIAL 1.5-V SSTL CLASS I" -to hps_memory_mem_dqs_n[0] -tag __hps_sdram_p0
	set_instance_assignment -name INPUT_TERMINATION "PARALLEL 50 OHM WITH CALIBRATION" -to hps_memory_mem_dqs_n[0] -tag __hps_sdram_p0
	set_instance_assignment -name OUTPUT_TERMINATION "SERIES 50 OHM WITH CALIBRATION" -to hps_memory_mem_dqs_n[0] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "DIFFERENTIAL 1.5-V SSTL CLASS I" -to hps_memory_mem_dqs_n[1] -tag __hps_sdram_p0
	set_instance_assignment -name INPUT_TERMINATION "PARALLEL 50 OHM WITH CALIBRATION" -to hps_memory_mem_dqs_n[1] -tag __hps_sdram_p0
	set_instance_assignment -name OUTPUT_TERMINATION "SERIES 50 OHM WITH CALIBRATION" -to hps_memory_mem_dqs_n[1] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "DIFFERENTIAL 1.5-V SSTL CLASS I" -to hps_memory_mem_dqs_n[2] -tag __hps_sdram_p0
	set_instance_assignment -name INPUT_TERMINATION "PARALLEL 50 OHM WITH CALIBRATION" -to hps_memory_mem_dqs_n[2] -tag __hps_sdram_p0
	set_instance_assignment -name OUTPUT_TERMINATION "SERIES 50 OHM WITH CALIBRATION" -to hps_memory_mem_dqs_n[2] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "DIFFERENTIAL 1.5-V SSTL CLASS I" -to hps_memory_mem_dqs_n[3] -tag __hps_sdram_p0
	set_instance_assignment -name INPUT_TERMINATION "PARALLEL 50 OHM WITH CALIBRATION" -to hps_memory_mem_dqs_n[3] -tag __hps_sdram_p0
	set_instance_assignment -name OUTPUT_TERMINATION "SERIES 50 OHM WITH CALIBRATION" -to hps_memory_mem_dqs_n[3] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "DIFFERENTIAL 1.5-V SSTL CLASS I" -to hps_memory_mem_ck -tag __hps_sdram_p0
	set_instance_assignment -name OUTPUT_TERMINATION "SERIES 50 OHM WITHOUT CALIBRATION" -to hps_memory_mem_ck -tag __hps_sdram_p0
	set_instance_assignment -name D5_DELAY 2 -to hps_memory_mem_ck -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "DIFFERENTIAL 1.5-V SSTL CLASS I" -to hps_memory_mem_ck_n -tag __hps_sdram_p0
	set_instance_assignment -name OUTPUT_TERMINATION "SERIES 50 OHM WITHOUT CALIBRATION" -to hps_memory_mem_ck_n -tag __hps_sdram_p0
	set_instance_assignment -name D5_DELAY 2 -to hps_memory_mem_ck_n -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_a[0] -tag __hps_sdram_p0
	set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to hps_memory_mem_a[0] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_a[10] -tag __hps_sdram_p0
	set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to hps_memory_mem_a[10] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_a[11] -tag __hps_sdram_p0
	set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to hps_memory_mem_a[11] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_a[12] -tag __hps_sdram_p0
	set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to hps_memory_mem_a[12] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_a[13] -tag __hps_sdram_p0
	set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to hps_memory_mem_a[13] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_a[14] -tag __hps_sdram_p0
	set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to hps_memory_mem_a[14] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_a[1] -tag __hps_sdram_p0
	set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to hps_memory_mem_a[1] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_a[2] -tag __hps_sdram_p0
	set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to hps_memory_mem_a[2] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_a[3] -tag __hps_sdram_p0
	set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to hps_memory_mem_a[3] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_a[4] -tag __hps_sdram_p0
	set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to hps_memory_mem_a[4] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_a[5] -tag __hps_sdram_p0
	set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to hps_memory_mem_a[5] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_a[6] -tag __hps_sdram_p0
	set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to hps_memory_mem_a[6] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_a[7] -tag __hps_sdram_p0
	set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to hps_memory_mem_a[7] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_a[8] -tag __hps_sdram_p0
	set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to hps_memory_mem_a[8] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_a[9] -tag __hps_sdram_p0
	set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to hps_memory_mem_a[9] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_ba[0] -tag __hps_sdram_p0
	set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to hps_memory_mem_ba[0] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_ba[1] -tag __hps_sdram_p0
	set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to hps_memory_mem_ba[1] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_ba[2] -tag __hps_sdram_p0
	set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to hps_memory_mem_ba[2] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_cas_n -tag __hps_sdram_p0
	set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to hps_memory_mem_cas_n -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_cke -tag __hps_sdram_p0
	set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to hps_memory_mem_cke -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_cs_n -tag __hps_sdram_p0
	set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to hps_memory_mem_cs_n -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_odt -tag __hps_sdram_p0
	set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to hps_memory_mem_odt -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_ras_n -tag __hps_sdram_p0
	set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to hps_memory_mem_ras_n -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_we_n -tag __hps_sdram_p0
	set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to hps_memory_mem_we_n -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_reset_n -tag __hps_sdram_p0
	set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to hps_memory_mem_reset_n -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_dm[0] -tag __hps_sdram_p0
	set_instance_assignment -name OUTPUT_TERMINATION "SERIES 50 OHM WITH CALIBRATION" -to hps_memory_mem_dm[0] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_dm[1] -tag __hps_sdram_p0
	set_instance_assignment -name OUTPUT_TERMINATION "SERIES 50 OHM WITH CALIBRATION" -to hps_memory_mem_dm[1] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_dm[2] -tag __hps_sdram_p0
	set_instance_assignment -name OUTPUT_TERMINATION "SERIES 50 OHM WITH CALIBRATION" -to hps_memory_mem_dm[2] -tag __hps_sdram_p0
	set_instance_assignment -name IO_STANDARD "SSTL-15 CLASS I" -to hps_memory_mem_dm[3] -tag __hps_sdram_p0
	set_instance_assignment -name OUTPUT_TERMINATION "SERIES 50 OHM WITH CALIBRATION" -to hps_memory_mem_dm[3] -tag __hps_sdram_p0
	set_instance_assignment -name GLOBAL_SIGNAL OFF -to soc_inst|hps_0|hps_io|border|hps_sdram_inst|p0|umemphy|ureset|phy_reset_mem_stable_n -tag __hps_sdram_p0
	set_instance_assignment -name GLOBAL_SIGNAL OFF -to soc_inst|hps_0|hps_io|border|hps_sdram_inst|p0|umemphy|ureset|phy_reset_n -tag __hps_sdram_p0
	set_instance_assignment -name GLOBAL_SIGNAL OFF -to soc_inst|hps_0|hps_io|border|hps_sdram_inst|p0|umemphy|uio_pads|dq_ddio[0].read_capture_clk_buffer -tag __hps_sdram_p0
	set_instance_assignment -name GLOBAL_SIGNAL OFF -to soc_inst|hps_0|hps_io|border|hps_sdram_inst|p0|umemphy|uread_datapath|reset_n_fifo_write_side[0] -tag __hps_sdram_p0
	set_instance_assignment -name GLOBAL_SIGNAL OFF -to soc_inst|hps_0|hps_io|border|hps_sdram_inst|p0|umemphy|uread_datapath|reset_n_fifo_wraddress[0] -tag __hps_sdram_p0
	set_instance_assignment -name GLOBAL_SIGNAL OFF -to soc_inst|hps_0|hps_io|border|hps_sdram_inst|p0|umemphy|uio_pads|dq_ddio[1].read_capture_clk_buffer -tag __hps_sdram_p0
	set_instance_assignment -name GLOBAL_SIGNAL OFF -to soc_inst|hps_0|hps_io|border|hps_sdram_inst|p0|umemphy|uread_datapath|reset_n_fifo_write_side[1] -tag __hps_sdram_p0
	set_instance_assignment -name GLOBAL_SIGNAL OFF -to soc_inst|hps_0|hps_io|border|hps_sdram_inst|p0|umemphy|uread_datapath|reset_n_fifo_wraddress[1] -tag __hps_sdram_p0
	set_instance_assignment -name GLOBAL_SIGNAL OFF -to soc_inst|hps_0|hps_io|border|hps_sdram_inst|p0|umemphy|uio_pads|dq_ddio[2].read_capture_clk_buffer -tag __hps_sdram_p0
	set_instance_assignment -name GLOBAL_SIGNAL OFF -to soc_inst|hps_0|hps_io|border|hps_sdram_inst|p0|umemphy|uread_datapath|reset_n_fifo_write_side[2] -tag __hps_sdram_p0
	set_instance_assignment -name GLOBAL_SIGNAL OFF -to soc_inst|hps_0|hps_io|border|hps_sdram_inst|p0|umemphy|uread_datapath|reset_n_fifo_wraddress[2] -tag __hps_sdram_p0
	set_instance_assignment -name GLOBAL_SIGNAL OFF -to soc_inst|hps_0|hps_io|border|hps_sdram_inst|p0|umemphy|uio_pads|dq_ddio[3].read_capture_clk_buffer -tag __hps_sdram_p0
	set_instance_assignment -name GLOBAL_SIGNAL OFF -to soc_inst|hps_0|hps_io|border|hps_sdram_inst|p0|umemphy|uread_datapath|reset_n_fifo_write_side[3] -tag __hps_sdram_p0
	set_instance_assignment -name GLOBAL_SIGNAL OFF -to soc_inst|hps_0|hps_io|border|hps_sdram_inst|p0|umemphy|uread_datapath|reset_n_fifo_wraddress[3] -tag __hps_sdram_p0
	set_instance_assignment -name ENABLE_BENEFICIAL_SKEW_OPTIMIZATION_FOR_NON_GLOBAL_CLOCKS ON -to soc_inst|hps_0|hps_io|border|hps_sdram_inst -tag __hps_sdram_p0
	set_instance_assignment -name PLL_COMPENSATION_MODE DIRECT -to soc_inst|hps_0|hps_io|border|hps_sdram_inst|pll0|fbout -tag __hps_sdram_p0
	set_instance_assignment -name PARTITION_HIERARCHY root_partition -to | -section_id Top

	# Commit assignments
	export_assignments

	# Close project
	if {$need_to_close_project} {
		project_close
	}
}
