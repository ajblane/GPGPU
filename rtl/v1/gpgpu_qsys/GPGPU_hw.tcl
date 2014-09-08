# TCL File Generated by Component Editor 14.0
# Mon Sep 08 21:03:21 CST 2014
# DO NOT MODIFY


# 
# GPGPU "GPGPU" v1.0
# ajblane 2014.09.08.21:03:21
# 
# 

# 
# request TCL package from ACDS 14.0
# 
package require -exact qsys 14.0


# 
# module GPGPU
# 
set_module_property DESCRIPTION ""
set_module_property NAME GPGPU
set_module_property VERSION 1.0
set_module_property INTERNAL false
set_module_property OPAQUE_ADDRESS_MAP true
set_module_property GROUP System
set_module_property AUTHOR ajblane
set_module_property DISPLAY_NAME GPGPU
set_module_property INSTANTIATE_IN_SYSTEM_MODULE true
set_module_property EDITABLE true
set_module_property REPORT_TO_TALKBACK false
set_module_property ALLOW_GREYBOX_GENERATION false
set_module_property REPORT_HIERARCHY false


# 
# file sets
# 
add_fileset QUARTUS_SYNTH QUARTUS_SYNTH "" ""
set_fileset_property QUARTUS_SYNTH TOP_LEVEL fpga_top
set_fileset_property QUARTUS_SYNTH ENABLE_RELATIVE_INCLUDE_PATHS false
set_fileset_property QUARTUS_SYNTH ENABLE_FILE_OVERWRITE_MODE true
add_fileset_file fpga_top.sv SYSTEM_VERILOG PATH ../fpga/fpga_top.sv TOP_LEVEL_FILE
add_fileset_file arbiter.sv SYSTEM_VERILOG PATH ../core/arbiter.sv
add_fileset_file cache_lru.sv SYSTEM_VERILOG PATH ../core/cache_lru.sv
add_fileset_file cache_valid_array.sv SYSTEM_VERILOG PATH ../core/cache_valid_array.sv
add_fileset_file cam.sv SYSTEM_VERILOG PATH ../core/cam.sv
add_fileset_file control_registers.sv SYSTEM_VERILOG PATH ../core/control_registers.sv
add_fileset_file core.sv SYSTEM_VERILOG PATH ../core/core.sv
add_fileset_file decode_stage.sv SYSTEM_VERILOG PATH ../core/decode_stage.sv
add_fileset_file defines.sv SYSTEM_VERILOG PATH ../core/defines.sv
add_fileset_file endian_swapper.sv SYSTEM_VERILOG PATH ../core/endian_swapper.sv
add_fileset_file execute_stage.sv SYSTEM_VERILOG PATH ../core/execute_stage.sv
add_fileset_file fp_adder_stage1.sv SYSTEM_VERILOG PATH ../core/fp_adder_stage1.sv
add_fileset_file fp_adder_stage2.sv SYSTEM_VERILOG PATH ../core/fp_adder_stage2.sv
add_fileset_file fp_adder_stage3.sv SYSTEM_VERILOG PATH ../core/fp_adder_stage3.sv
add_fileset_file fp_multiplier_stage1.sv SYSTEM_VERILOG PATH ../core/fp_multiplier_stage1.sv
add_fileset_file fp_normalize.sv SYSTEM_VERILOG PATH ../core/fp_normalize.sv
add_fileset_file fp_reciprocal_estimate.sv SYSTEM_VERILOG PATH ../core/fp_reciprocal_estimate.sv
add_fileset_file gpgpu.sv SYSTEM_VERILOG PATH ../core/gpgpu.sv
add_fileset_file instruction_fetch_stage.sv SYSTEM_VERILOG PATH ../core/instruction_fetch_stage.sv
add_fileset_file instruction_pipeline.sv SYSTEM_VERILOG PATH ../core/instruction_pipeline.sv
add_fileset_file integer_multiplier.sv SYSTEM_VERILOG PATH ../core/integer_multiplier.sv
add_fileset_file l1_cache.sv SYSTEM_VERILOG PATH ../core/l1_cache.sv
add_fileset_file l1_cache_tag.sv SYSTEM_VERILOG PATH ../core/l1_cache_tag.sv
add_fileset_file l1_load_miss_queue.sv SYSTEM_VERILOG PATH ../core/l1_load_miss_queue.sv
add_fileset_file l2_cache.sv SYSTEM_VERILOG PATH ../core/l2_cache.sv
add_fileset_file l2_cache_arb.sv SYSTEM_VERILOG PATH ../core/l2_cache_arb.sv
add_fileset_file l2_cache_bus_interface.sv SYSTEM_VERILOG PATH ../core/l2_cache_bus_interface.sv
add_fileset_file l2_cache_dir.sv SYSTEM_VERILOG PATH ../core/l2_cache_dir.sv
add_fileset_file l2_cache_pending_miss.sv SYSTEM_VERILOG PATH ../core/l2_cache_pending_miss.sv
add_fileset_file l2_cache_read.sv SYSTEM_VERILOG PATH ../core/l2_cache_read.sv
add_fileset_file l2_cache_response.sv SYSTEM_VERILOG PATH ../core/l2_cache_response.sv
add_fileset_file l2_cache_tag.sv SYSTEM_VERILOG PATH ../core/l2_cache_tag.sv
add_fileset_file l2_cache_write.sv SYSTEM_VERILOG PATH ../core/l2_cache_write.sv
add_fileset_file l2req_arbiter_mux.sv SYSTEM_VERILOG PATH ../core/l2req_arbiter_mux.sv
add_fileset_file mask_unit.sv SYSTEM_VERILOG PATH ../core/mask_unit.sv
add_fileset_file memory_access_stage.sv SYSTEM_VERILOG PATH ../core/memory_access_stage.sv
add_fileset_file multi_stage_alu.sv SYSTEM_VERILOG PATH ../core/multi_stage_alu.sv
add_fileset_file multiplexer.sv SYSTEM_VERILOG PATH ../core/multiplexer.sv
add_fileset_file one_hot_to_index.sv SYSTEM_VERILOG PATH ../core/one_hot_to_index.sv
add_fileset_file performance_counters.sv SYSTEM_VERILOG PATH ../core/performance_counters.sv
add_fileset_file reciprocal_rom.sv SYSTEM_VERILOG PATH ../core/reciprocal_rom.sv
add_fileset_file rollback_controller.sv SYSTEM_VERILOG PATH ../core/rollback_controller.sv
add_fileset_file scalar_register_file.sv SYSTEM_VERILOG PATH ../core/scalar_register_file.sv
add_fileset_file single_stage_alu.sv SYSTEM_VERILOG PATH ../core/single_stage_alu.sv
add_fileset_file sram_1r1w.sv SYSTEM_VERILOG PATH ../core/sram_1r1w.sv
add_fileset_file store_buffer.sv SYSTEM_VERILOG PATH ../core/store_buffer.sv
add_fileset_file strand_fsm.sv SYSTEM_VERILOG PATH ../core/strand_fsm.sv
add_fileset_file strand_select_stage.sv SYSTEM_VERILOG PATH ../core/strand_select_stage.sv
add_fileset_file sync_fifo.sv SYSTEM_VERILOG PATH ../core/sync_fifo.sv
add_fileset_file vector_bypass_unit.sv SYSTEM_VERILOG PATH ../core/vector_bypass_unit.sv
add_fileset_file vector_register_file.sv SYSTEM_VERILOG PATH ../core/vector_register_file.sv
add_fileset_file writeback_stage.sv SYSTEM_VERILOG PATH ../core/writeback_stage.sv
add_fileset_file async_fifo.sv SYSTEM_VERILOG PATH ../../fpga_common/async_fifo.sv
add_fileset_file axi_async_bridge.sv SYSTEM_VERILOG PATH ../../fpga_common/axi_async_bridge.sv
add_fileset_file axi_interconnect.sv SYSTEM_VERILOG PATH ../../fpga_common/axi_interconnect.sv
add_fileset_file axi_internal_ram.sv SYSTEM_VERILOG PATH ../../fpga_common/axi_internal_ram.sv


# 
# parameters
# 


# 
# display items
# 


# 
# connection point clock
# 
add_interface clock clock end
set_interface_property clock clockRate 0
set_interface_property clock ENABLED true
set_interface_property clock EXPORT_OF ""
set_interface_property clock PORT_NAME_MAP ""
set_interface_property clock CMSIS_SVD_VARIABLES ""
set_interface_property clock SVD_ADDRESS_GROUP ""


# 
# connection point reset
# 
add_interface reset reset end
set_interface_property reset associatedClock clock
set_interface_property reset synchronousEdges DEASSERT
set_interface_property reset ENABLED true
set_interface_property reset EXPORT_OF ""
set_interface_property reset PORT_NAME_MAP ""
set_interface_property reset CMSIS_SVD_VARIABLES ""
set_interface_property reset SVD_ADDRESS_GROUP ""


# 
# connection point avalon_slave_0
# 
add_interface avalon_slave_0 avalon end
set_interface_property avalon_slave_0 addressUnits WORDS
set_interface_property avalon_slave_0 associatedClock clock
set_interface_property avalon_slave_0 associatedReset ""
set_interface_property avalon_slave_0 bitsPerSymbol 8
set_interface_property avalon_slave_0 burstOnBurstBoundariesOnly false
set_interface_property avalon_slave_0 burstcountUnits WORDS
set_interface_property avalon_slave_0 explicitAddressSpan 0
set_interface_property avalon_slave_0 holdTime 0
set_interface_property avalon_slave_0 linewrapBursts false
set_interface_property avalon_slave_0 maximumPendingReadTransactions 0
set_interface_property avalon_slave_0 maximumPendingWriteTransactions 0
set_interface_property avalon_slave_0 readLatency 0
set_interface_property avalon_slave_0 readWaitTime 1
set_interface_property avalon_slave_0 setupTime 0
set_interface_property avalon_slave_0 timingUnits Cycles
set_interface_property avalon_slave_0 writeWaitTime 0
set_interface_property avalon_slave_0 ENABLED true
set_interface_property avalon_slave_0 EXPORT_OF ""
set_interface_property avalon_slave_0 PORT_NAME_MAP ""
set_interface_property avalon_slave_0 CMSIS_SVD_VARIABLES ""
set_interface_property avalon_slave_0 SVD_ADDRESS_GROUP ""

add_interface_port avalon_slave_0 awaddr readdata Output 32
add_interface_port avalon_slave_0 awlen readdata Output 8
add_interface_port avalon_slave_0 awvalid writeresponsevalid_n Output 1
add_interface_port avalon_slave_0 awready writeresponserequest_n Input 1
add_interface_port avalon_slave_0 wdata readdata Output 32
add_interface_port avalon_slave_0 wlast writeresponsevalid_n Output 1
add_interface_port avalon_slave_0 wvalid writeresponsevalid_n Output 1
add_interface_port avalon_slave_0 wready writeresponserequest_n Input 1
add_interface_port avalon_slave_0 bvalid writeresponserequest_n Input 1
add_interface_port avalon_slave_0 bready writeresponsevalid_n Output 1
add_interface_port avalon_slave_0 araddr readdata Output 32
add_interface_port avalon_slave_0 arlen readdata Output 8
add_interface_port avalon_slave_0 arvalid writeresponsevalid_n Output 1
add_interface_port avalon_slave_0 arready writeresponserequest_n Input 1
add_interface_port avalon_slave_0 rready writeresponsevalid_n Output 1
add_interface_port avalon_slave_0 rvalid writeresponserequest_n Input 1
add_interface_port avalon_slave_0 rdata writebyteenable_n Input 32
add_interface_port avalon_slave_0 clk50 writeresponserequest_n Input 1
add_interface_port avalon_slave_0 dram_dq export Bidir 32
set_interface_assignment avalon_slave_0 embeddedsw.configuration.isFlash 0
set_interface_assignment avalon_slave_0 embeddedsw.configuration.isMemoryDevice 0
set_interface_assignment avalon_slave_0 embeddedsw.configuration.isNonVolatileStorage 0
set_interface_assignment avalon_slave_0 embeddedsw.configuration.isPrintableDevice 0

