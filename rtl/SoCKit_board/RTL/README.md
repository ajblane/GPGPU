Add a AXI bus to control the processor from another processor(EX: cotrex-A9).

|bus address|mean|
|--------|-----|
|0|RESET|
|1|PC|

If the AXI bus is lightweight HPS2FPGA AXI bridge of HPS, the developer can set 1 at address 0xff200000 to control Nyuzi processor to start and set code base address of Nyuzi processor at address 0xff200004 to fetch instructions. PC is set, before the RESET signal is set. 
