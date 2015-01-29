// $File: axi_registers.sv 
// $Revision: #1 
// $Date: 2014/11/26 
// $Author: aj 
//-----------------------------------------------------------------------------
// GPGPU with AXI Slave Interface and AXI Master Interface
//-----------------------------------------------------------------------------
`include "defines.sv"
module axi_gpgpu (
			       clk,   
			       reset_n,
            //AXI Slave Interface for GPGPU setting  
			       axs_awid,
			       axs_awaddr,
			       axs_awlen,
			       axs_awsize,
			       axs_awburst,
			       axs_awlock,
			       axs_awcache,
			       axs_awprot,
			       axs_awvalid,
			       axs_awready,

			       axs_wid,
			       axs_wdata,
			       axs_wstrb,
			       axs_wlast,
			       axs_wvalid,
			       axs_wready,

			       axs_bid,
			       axs_bresp,
			       axs_bvalid,
			       axs_bready,
				  
			       axs_arid,
			       axs_araddr,
			       axs_arlen,
			       axs_arsize,
			       axs_arburst,
			       axs_arlock,
			       axs_arcache,
			       axs_arprot,
			       axs_arvalid,
			       axs_arready,

			       axs_rid,
			       axs_rdata,
			       axs_rlast,
			       axs_rvalid,
			       axs_rready,
			       axs_rresp,
 // AXI external memory interface
                               axm_awid,
			       axm_awaddr,
			       axm_awlen,
			       axm_awsize,
			       axm_awburst,
			       axm_awlock,
			       axm_awcache,
			       axm_awprot,
			       axm_awvalid,
			       axm_awready,

			       axm_wid,
			       axm_wdata,
			       axm_wstrb,
			       axm_wlast,
			       axm_wvalid,
			       axm_wready,

			       axm_bid,
			       axm_bresp,
			       axm_bvalid,
			       axm_bready,
				  
			       axm_arid,
			       axm_araddr,
			       axm_arlen,
			       axm_arsize,
			       axm_arburst,
			       axm_arlock,
			       axm_arcache,
			       axm_arprot,
			       axm_arvalid,
			       axm_arready,

			       axm_rid,
			       axm_rdata,
			       axm_rlast,
			       axm_rvalid,
			       axm_rready,
			       axm_rresp);

   // PARAMETERS for AXI --------------------------------------------
   parameter  SLAVE_AXI_ID_W            = 4;  // width of ID fields
   parameter  SLAVE_AXI_ADDRESS_W       = 6; // address width
   parameter  SLAVE_AXI_DATA_W          = 32; // data symbol width 
   parameter  SLAVE_AXI_NUMBYTES        = 4;  // number of bytes per word
   parameter  MAST_AXI_ID_W             = 4;

  



   //AXI write address bus ------------------------------------------
   output   [MAST_AXI_ID_W-1:0]      axm_awid;
   output   [31:0] axm_awaddr;
   output   [ 3:0]              axm_awlen;  //burst length is 1 + (0 - 15)
   output   [ 2:0]              axm_awsize;  //size of each transfer in burst
   output   [ 1:0]              axm_awburst; //for bursts>1, accept only incr burst=01
   output   [ 1:0]              axm_awlock;  //only normal access supported axs_awlock=00
   output   [ 3:0]              axm_awcache; 
   output   [ 2:0]              axm_awprot;
   output                       axm_awvalid; //master addr valid
   input                      axm_awready; //slave ready to accept
   
   //AXI write data bus ----------------------------------------------
   output   [MAST_AXI_ID_W-1:0]      axm_wid;
   output   [31:0]    axm_wdata;
   output   [SLAVE_AXI_NUMBYTES-1:0]  axm_wstrb;   //1 strobe per byte
   output                       axm_wlast;   //last transfer in burst
   output                       axm_wvalid;  //master data valid
   input                      axm_wready;  //slave ready to accept
   
   //AXI write response bus ------------------------------------------
   input  [MAST_AXI_ID_W-1:0]      axm_bid;
   input  [ 1:0]              axm_bresp;
   input                      axm_bvalid;
   output                       axm_bready;
   
   //AXI read address bus --------------------------------------------
   output   [MAST_AXI_ID_W-1:0]      axm_arid;
   output   [31-1:0] axm_araddr;
   output   [ 3:0]              axm_arlen;   //burst length - 1 to 16
   output   [ 2:0]              axm_arsize;  //size of each transfer in burst
   output   [ 1:0]              axm_arburst; //for bursts>1, accept only incr burst=01
   output   [ 1:0]              axm_arlock;  //only normal access supported axs_awlock=00
   output   [ 3:0]              axm_arcache; 
   output   [ 2:0]              axm_arprot;
   output                       axm_arvalid; //master addr valid
   input                      axm_arready; //slave ready to accept
   
   //AXI read data bus -----------------------------------------------
   input  [MAST_AXI_ID_W-1:0]      axm_rid;
   input  [31:0]    axm_rdata;
   input  [ 1:0]              axm_rresp;
   input                      axm_rlast; //last transfer in burst
   input                      axm_rvalid;//slave data valid
   output                     axm_rready; //master ready to accept

   // ---------------------------------------------------------------
   // I/O Port Declarations -----------------------------------------
   // ---------------------------------------------------------------
   input                      clk;
   input                      reset_n;

   //AXI write address bus ------------------------------------------
   input   [SLAVE_AXI_ID_W-1:0]      axs_awid;
   input   [SLAVE_AXI_ADDRESS_W-1:0] axs_awaddr;
   input   [ 3:0]              axs_awlen;   //burst length is 1 + (0 - 15)
   input   [ 2:0]              axs_awsize;  //size of each transfer in burst
   input   [ 1:0]              axs_awburst; //for bursts>1, accept only incr burst=01
   input   [ 1:0]              axs_awlock;  //only normal access supported axs_awlock=00
   input   [ 3:0]              axs_awcache; 
   input   [ 2:0]              axs_awprot;
   input                       axs_awvalid; //master addr valid
   output                      axs_awready; //slave ready to accept

   //AXI write data bus ---------------------------------------------
   input   [SLAVE_AXI_ID_W-1:0]      axs_wid;
   input   [SLAVE_AXI_DATA_W-1:0]    axs_wdata;
   input   [SLAVE_AXI_NUMBYTES-1:0]  axs_wstrb;   //1 strobe per byte
   input                       axs_wlast;   //last transfer in burst
   input                       axs_wvalid;  //master data valid
   output                      axs_wready;  //slave ready to accept

   //AXI write response bus -----------------------------------------
   output  [SLAVE_AXI_ID_W-1:0]      axs_bid;
   output  [ 1:0]              axs_bresp;
   output                      axs_bvalid;
   input                       axs_bready;
   
   //AXI read address bus -------------------------------------------
   input   [SLAVE_AXI_ID_W-1:0]      axs_arid;
   input   [SLAVE_AXI_ADDRESS_W-1:0] axs_araddr;
   input   [ 3:0]              axs_arlen;   //burst length - 1 to 16
   input   [ 2:0]              axs_arsize;  //size of each transfer in burst
   input   [ 1:0]              axs_arburst; //for bursts>1, accept only incr burst=01
   input   [ 1:0]              axs_arlock;  //only normal access supported axs_awlock=00
   input   [ 3:0]              axs_arcache; 
   input   [ 2:0]              axs_arprot;
   input                       axs_arvalid; //master addr valid
   output                      axs_arready; //slave ready to accept

   //AXI read data bus ----------------------------------------------
   output  [SLAVE_AXI_ID_W-1:0]      axs_rid;
   output  [SLAVE_AXI_DATA_W-1:0]    axs_rdata;
   output  [ 1:0]              axs_rresp;
   output                      axs_rlast; //last transfer in burst
   output                      axs_rvalid;//slave data valid
   input                       axs_rready;//master ready to accept

   //The variables xsomething (i.e. xwaddr, xwlen, xwaddval etc) are locally stored
   //values from axi bus transactions.  For instance, read requests can come
   //in faster than they can be serviced, expecially with burst reads.  A read
   //access with axs_rid = 7 will store bus values in xraddr[7], and xrlen[7].
   //The bit xraddval[7] indicates that the xraddr[7] and xrlen[7] values are valid
   //i.e. they represent reads that need to be serviced. 
   
   // ---------------------------------------------------------------
   // Variable Declarations -----------------------------------------
   // ---------------------------------------------------------------
   integer i;  //loop index - various places
   // axi waddr logic -----------------------------------------------
   logic [SLAVE_AXI_ADDRESS_W-1:0]  xwaddr [15:0];
   logic [ 4:0]               xwlen  [15:0];
   logic [15:0]               xwaddval, clrwaddval;
   logic [ 2:0]               xwsize;
   logic [ 1:0]               xwburst;
   logic                      axs_awready;

   // axi wdata logic -----------------------------------------------
   logic [SLAVE_AXI_DATA_W-1:0]     xwdata [15:0];
   logic [SLAVE_AXI_DATA_W/8-1:0]   xwstrb [15:0];
   logic [15:0]               xwlast;
   logic [15:0]               xwdataval;
   logic                      axs_wready;
   logic [15:0]               inc_waddr;
   logic [SLAVE_AXI_ID_W-1:0]       nx_windex, windex;
   logic   [1:0]              wstate, nx_wstate;
   logic                      ctl_write, ctl_write_done;
   logic                      save_wdata;
   logic                      nx_wready;
   logic  [15:0]              nx_wdataval;
   logic  [SLAVE_AXI_ADDRESS_W-1:0] scr_wr_addr; //scr_signals go to the
   logic  [SLAVE_AXI_DATA_W-1:0]    scr_din;     //RAM instance
   logic  [SLAVE_AXI_NUMBYTES-1:0]  scr_we;


   // axi write response logic --------------------------------------
   logic [SLAVE_AXI_ID_W-1:0]       axs_bid;
   logic [1:0]                axs_bresp;
   logic                      axs_bvalid;
   logic [SLAVE_AXI_ID_W-1:0]       pid;
   logic                      wrt_resp;
   logic                      wrt_ok;
   
   // axi raddr logic -----------------------------------------------
   logic [SLAVE_AXI_ADDRESS_W-1:0]  xraddr [15:0];
   logic [ 4:0]               xrlen  [15:0];
   logic [15:0]               xraddval;
   logic [ 2:0]               xrsize;
   logic [ 1:0]               xrburst;
   logic                      axs_arready;

   // axi rdata logic -----------------------------------------------
   logic                      axs_rlast;
   logic                      axs_rvalid;
   logic                      nx_rvalid;
   logic                      nx_rlast;
   logic [SLAVE_AXI_ID_W-1:0]       nx_rid;
   logic [SLAVE_AXI_DATA_W-1:0]     ppl_buff;
   logic [SLAVE_AXI_DATA_W-1:0]     axs_rdata;
   logic [ 1:0]               axs_rresp;
   logic [SLAVE_AXI_ID_W-1:0]       axs_rid;
   logic [15:0]               clr_raddval;
   logic [SLAVE_AXI_ID_W-1:0]       nx_rindex, rindex;
   logic [ 1:0]               rstate, nx_rstate;
   logic [SLAVE_AXI_DATA_W-1:0]     scr_dout; 
   logic [SLAVE_AXI_ADDRESS_W-1:0]  scr_rd_addr;
   logic                      sgrant;
   logic                      nextword;
   logic [ 4:0]               fcount, fcount_d;
   
   logic                  registers_valid;
   logic [SLAVE_AXI_DATA_W-1:0] ini_registers[2]; 
   // [0] RESET signal
   // [1] code base address
   axi_interface axi_bus_core();


// AXI write address logic ------------------------------------------
   always @(posedge clk or negedge reset_n) begin
      if (~reset_n) begin
         axs_awready     <= 1'b0;
         for (i=0; i<16; i++) begin
            xwaddr[i]    <= 0;
            xwlen[i]     <= 5'h0;
         end
         xwaddval        <= 16'h0;
         xwsize          <= 3'h0;
         xwburst         <= 2'h0;
      end
      else begin
         axs_awready <= (xwaddval == 16'h0);
         xwsize      <= xwsize;
         xwburst     <= xwburst;
         if (clrwaddval!=16'h0) begin //clr addval bits on last write of data
            for (i=0; i<16; i++) begin
               if (clrwaddval[i]) xwaddval[i] <= 1'b0;
            end
         end
         else if (inc_waddr!=16'h0) begin //inc addresses with each write
            for (i=0; i<16; i++) begin
               if (inc_waddr[i]) begin
                  xwlen[i]  <= xwlen[i] - 1'b1;
                  if (axs_wstrb[3]) xwaddr[i] <= xwaddr[i] + 1'b1;
               end
            end
         end          
         else if (axs_awvalid && xwaddval[axs_awid]==1'b0) begin //store waddr parameters
            xwaddr[axs_awid]    <= axs_awaddr;  //save the write address
            xwaddval[axs_awid]  <= 1'b1;        //set valid bit
            xwlen[axs_awid]     <= axs_awlen + 1'b1;   //save the burst length
            xwsize              <= axs_awsize;  //save the burst size (bytes per transfer)
            xwburst             <= axs_awburst; //burst type - 00 fixed/ 01 incrementing
            axs_awready         <= ~axs_awready;//if high, xaction done, switch low
         end                                    //if low, switch high 1 cycle to end xaction
      end
   end

   //AXI write data logic
   localparam W_IDLE       = 2'h0;
   localparam W_MEM        = 2'h1;
   localparam W_BRST       = 2'h2;
   localparam W_BUF        = 2'h3;
      
   always @(posedge clk or negedge reset_n) begin
      if (~reset_n) begin
         axs_wready        <=  1'b0;
         ctl_write_done    <=  1'b0;
         windex            <= {SLAVE_AXI_ID_W{1'b0}};
         wstate            <= W_IDLE;
         xwdataval         <= 16'h0;
         xwlast            <= 16'h0;
         for (i=0; i<16; i++) begin
            xwdata[i]      <= {SLAVE_AXI_DATA_W{1'b0}};
            xwstrb[i]      <= {SLAVE_AXI_NUMBYTES{1'b0}};
         end
      end
      else begin
         axs_wready            <= nx_wready;
         ctl_write_done        <= 1'b0;
         windex                <= nx_windex;
         wstate                <= nx_wstate;
         xwdataval             <= nx_wdataval;
         if (save_wdata) begin
            xwdata[axs_wid]    <= axs_wdata;
            xwstrb[axs_wid]    <= axs_wstrb;
            xwdataval[axs_wid] <= 1'b1;
            xwlast[axs_wid]    <= axs_wlast;
         end
      end
   end

   always @ (*) begin
	   clrwaddval         = 16'h0;
      ctl_write          =  1'b0;
      i                  = 32'h0;
      inc_waddr          = 16'h0;
      nx_wstate          = wstate;
      nx_wdataval        = xwdataval;
      nx_windex          = windex;
      nx_wready          =  1'b0;
      pid                = {SLAVE_AXI_ID_W{1'b0}};
      save_wdata         =  1'b0;
      scr_din            = {SLAVE_AXI_DATA_W{1'b0}};
      scr_we             = {SLAVE_AXI_NUMBYTES{1'b0}};
      scr_wr_addr        = {(SLAVE_AXI_ADDRESS_W){1'b0}};
      wrt_ok             =  1'b0;
      wrt_resp           =  1'b0;
      case (wstate)
         W_IDLE: begin  //s0
            //write to memory (address valid)
            if (axs_wvalid && xwaddval[axs_wid]) begin
               nx_wstate          = W_MEM;
               scr_wr_addr        = xwaddr[axs_wid];
               scr_din            = axs_wdata;
               scr_we             = axs_wstrb;
               nx_wready          = 1'b1;
	            inc_waddr[axs_wid] = ~axs_wlast;
	            nx_windex          = axs_wid;
	         end
	         //write addr not ready, so save write data for later
	         else if (axs_wvalid && ~xwdataval[axs_wid]) begin
	            nx_wstate         = W_BUF;
	            save_wdata        = 1'b1;
	            nx_wready         = 1'b1;
	         end
	         //check to see if xwaddval[i] true for saved wdata
	         else begin
	            for (i=0; i<16; i++) begin
	               if (xwdataval[i] && xwaddval[i]) begin
	                  nx_windex    = i;
	                  nx_wstate    = W_MEM;
	                  scr_wr_addr  = xwaddr[i];
	                  scr_din      = xwdata[i];
	                  scr_we       = xwstrb[i];
	                  if (xwlast[i]) begin //if last write
	                     nx_wdataval[i] = 1'b0;
	                     clrwaddval[i]  = 1'b1;
	                     pid            = i;
	                     wrt_resp       = 1'b1;
	                     wrt_ok         = (xwlen[i]==5'h1);
	                  end
	               end
	            end //for
	         end //else
	      end //W_IDLE
	      W_MEM: begin // s1
	      //do one write, then jump to write burst or back to idle
	         if (axs_wvalid && (xwlen[windex]>5'h0) && ~axs_wlast) begin
	            nx_wstate          = W_BRST;
               scr_wr_addr        = xwaddr[windex];
               scr_din            = axs_wdata;
               nx_wready          = 1'b1;
            end
            else begin //last write (only 1 word written)
	            nx_wstate    = W_IDLE;
	            nx_wdataval[windex]   = 1'b0; 
	            clrwaddval[windex]    = 1'b1;
	            pid                   = windex;
	            wrt_resp              = 1'b1;
	            wrt_ok                = ctl_write_done || (xwlen[windex]==5'h1);
	         end
	      end
	      W_BRST: begin // s2
            if ((xwlen[windex]>5'h0) && ~axs_wlast) begin
	            nx_wstate           = W_BRST;
               scr_wr_addr         = xwaddr[windex];
               scr_din             = axs_wdata;
               scr_we              = {4{axs_wvalid}} & axs_wstrb;
               inc_waddr[windex]   = axs_wvalid;
               nx_wready           = 1'b1;
	         end
	         else begin //last write of burst
	            nx_wstate           = W_IDLE;
               scr_wr_addr         = xwaddr[windex];
               scr_din             = axs_wdata;
	            scr_we              = {4{axs_wvalid}} & axs_wstrb;
               inc_waddr[windex]   = 1'b0;
	            clrwaddval[windex]  = ((xwlen[windex]==5'h1) || axs_wlast);
	            pid                 = windex;
	            wrt_resp            = 1'b1;
	            wrt_ok              = (xwlen[windex]==5'h1) && axs_wlast;

	         end
	      end
	      W_BUF: begin // s3
	         nx_wstate    = W_IDLE;
	      end
	   endcase
	end


   //AXI write response logic
   always @(posedge clk or negedge reset_n) begin
      if (~reset_n) begin
         axs_bid      <= {SLAVE_AXI_ID_W{1'b0}};
         axs_bresp    <= 2'h0;
         axs_bvalid   <= 1'b0;
      end
      else if (axs_bvalid && axs_bready) begin
	      // synthesis translate_off
	      if (axs_bresp==2'h0)
	         $sformat(message, "%m: - write response OK");
	      else
	         $sformat(message, "%m: - write response SLV_ERR");
	      print(VERBOSITY_DEBUG, message);  	          
	      // synthesis translate_on   		 
         axs_bid      <= {SLAVE_AXI_ID_W{1'b0}};
         axs_bresp    <= 2'h0;
         axs_bvalid   <= 1'b0;
      end
      else if (wrt_resp) begin
         axs_bid      <= pid;
         if (wrt_ok ) axs_bresp <= 2'h0; //OK
         else         axs_bresp <= 2'h2; //slverr
         axs_bvalid   <= 1'b1;
      end
   end
         
   //AXI read address logic
   always @(posedge clk or negedge reset_n) begin
      if (~reset_n) begin
         axs_arready <= 1'b0;
         for (i=0; i<16; i++) begin
            xraddr[i] <= 0;
            xrlen[i]  <= 5'h0;
         end
         xraddval      <= 16'h0;
         xrsize        <= 3'h0;
         xrburst       <= 2'h0;
      end
      else begin
         axs_arready <= (xraddval == 16'h0);
         if (clr_raddval != 16'h0) begin // clear address valid bit
            for (i=0; i<16; i++) begin
               if (clr_raddval[i]) xraddval[i] <= 1'b0;
            end
         end
         else if (axs_arvalid && xraddval[axs_arid]==1'b0) begin //store raddr parameters
            xraddr[axs_arid]    <= axs_araddr;  //save the write address
            xraddval[axs_arid]  <= 1'b1;        //set valid bit
            xrlen[axs_arid]     <= axs_arlen + 1'b1;   //save the burst length
            xrsize              <= axs_arsize;  //save the burst size (bytes per transfer)
            xrburst             <= axs_arburst; //burst type - 00 fixed/ 01 incrementing
            axs_arready         <= ~axs_arready;//if high, xaction done, switch low
                                                //if low, switch high 1 cycle to end xaction  		 
         end
      end
   end

   
   //AXI read data logic
   localparam R_IDLE = 2'h0;
   localparam R_PAUS = 2'h1;
   localparam R_BST  = 2'h2;
   localparam R_LAST = 2'h3;

   always @(posedge clk or negedge reset_n) begin
      if (~reset_n) begin
         rstate     <= R_IDLE;
         axs_rvalid <=  1'b0;
         axs_rresp  <=  2'h0;
         axs_rid    <= {SLAVE_AXI_ID_W{1'b0}};
         axs_rlast  <=  1'b0;
         ppl_buff   <= {SLAVE_AXI_DATA_W{1'b0}};
         axs_rdata  <= {SLAVE_AXI_DATA_W{1'b0}};
         rindex     <= {SLAVE_AXI_ID_W{1'b0}};
         sgrant     <=  1'b0;
         fcount_d   <=  5'h0;
      end
      else begin
         rstate     <= nx_rstate;
         axs_rvalid <= nx_rvalid;
         axs_rresp  <=  2'h0; //always set to 00 for now ...
         axs_rid    <= nx_rid;
         axs_rlast  <= nx_rlast;
         rindex     <= nx_rindex;
         fcount_d   <= fcount;
         if (nextword) begin
            ppl_buff   <= scr_dout;
            axs_rdata  <= ppl_buff;
         end
         else if (rstate==R_IDLE) begin
            ppl_buff   <= {SLAVE_AXI_DATA_W{1'b0}};
            axs_rdata  <= {SLAVE_AXI_DATA_W{1'b0}};
         end
      end
   end


   always @ (*) begin
      nx_rstate         =  rstate;
      nx_rvalid         =  axs_rvalid;
      nx_rlast          =  axs_rlast;
      nx_rid            =  axs_rid;
      nx_rindex         =  rindex;
      clr_raddval       =  16'h0;
      i                 =  32'h0;
      nextword          =   1'b0;
      scr_rd_addr       =  {SLAVE_AXI_ADDRESS_W{1'b0}};
      fcount            =  fcount_d;
      case (rstate)
         R_IDLE: begin
          for (i=0; i<16; i++) begin //memory reads
               if (xraddval[i]) begin 
                  nx_rstate    = R_PAUS;
                  scr_rd_addr  = xraddr[i];
                  nx_rindex    = i;
                  fcount       = 5'h0;
               end
            end
         end
         R_PAUS: begin //pause to get some data
            fcount       = fcount_d + 1'b1;
            scr_rd_addr  = xraddr[rindex] + fcount;
            nextword     = 1'b1;

            if (fcount==5'h2) begin
              nx_rid       = rindex;
              nx_rstate    = R_BST;
              nx_rvalid    = 1'b1;
              fcount       = 5'h2;
              if (xrlen[rindex] == 5'h1) begin
                 nx_rstate  = R_LAST;
                 nx_rlast   = 1'b1;
              end;
            end
         end
         R_BST: begin
            if (axs_rready) begin
               nextword = 1'b1;
               fcount   = fcount_d + 1'b1;
               if (fcount == xrlen[rindex] + 1'b1) begin
                  nx_rstate  = R_LAST;
                  nx_rlast   = 1'b1;
               end
            end
            scr_rd_addr = xraddr[rindex] + fcount;
         end
         R_LAST: begin
            scr_rd_addr    = {SLAVE_AXI_ADDRESS_W{1'b0}};
               nextword    = 1'b0;
            if (axs_rready) begin
               nx_rstate   = R_IDLE;
               nx_rvalid   = 1'b0;
               nx_rid      = {SLAVE_AXI_ID_W{1'b0}};
               nx_rlast    = 1'b0;
               clr_raddval[rindex] = 1'b1;
            end
         end
      endcase
   end
	

//ini_registers 
always @ (posedge clk or negedge reset_n) begin
   if(~reset_n)begin
      ini_registers[0] <= 32'b0; // reset = 0 
      ini_registers[1] <= 32'b0; // PC =0
      registers_valid <= 1'b0;
   end
   else if(scr_we)begin
         ini_registers[scr_wr_addr] <= scr_din;
         registers_valid <= 1'b0; 
   end
   else if(ini_registers[0] == 32'b1)begin // active high keeps 2 cycles for nyuzi processor 
       ini_registers[0] <= 32'b0;
   end
end

	nyuzi nyuzi(
	  .interrupt_req(0),
	   // Interfaces
	  .axi_bus		(axi_bus_core),
           // Outputs
	  .processor_halt	(),
	  .io_write_en	        (),
	  .io_read_en		(),
	  .io_address		(),
	  .io_write_data	(),
	  // Inputs
	  .clk		        (clk),
	  .reset		(ini_registers[0]),
	  .io_read_data	        (),
          .reset_pc             (ini_registers[1]));

assign axm_awaddr  =axi_bus_core.awaddr;//master       
assign axm_awlen   =axi_bus_core.awlen  ; // master
assign axm_awvalid =axi_bus_core.awvalid; // master
assign axi_bus_core.awready =axm_awready; // slave
assign axm_wdata   =axi_bus_core.wdata  ; // master
assign axm_wlast   =axi_bus_core.wlast  ; // master
assign axm_wvalid  =axi_bus_core.wvalid ; // master
assign axi_bus_core.wready  =axm_wready ; // slave
assign axi_bus_core.bvalid  =axm_bvalid ; // slave
assign axm_bready  =axi_bus_core.bready ; // master
assign axm_araddr  =axi_bus_core.araddr ; // master
assign axm_arlen   =axi_bus_core.arlen  ; // master
assign axm_arvalid =axi_bus_core.arvalid; // master
assign axi_bus_core.arready =axm_arready; // slave
assign axm_rready  =axi_bus_core.rready ; // master
assign axi_bus_core.rvalid  = axm_rvalid ; // slave
assign axi_bus_core.rdata   = axm_rdata  ; // slave






endmodule
   

