`define WORD_SIZE 16
`define FETCH_SIZE 64
/*************************************************
* DMA module (DMA.v)
* input: clock (CLK), bus grant (BG) signal, 
*        data from the device (edata), and DMA command (cmd)
* output: bus request (BR) signal 
*         WRITE signal
*         memory address (addr) to be written by the device, 
*         offset device offset (0 - 2)
*         data that will be written to the memory
*         interrupt to notify DMA is end
* You should NOT change the name of the I/O ports and the module name
* You can (or may have to) change the type and length of I/O ports 
* (e.g., wire -> reg) if you want 
* Do not add more ports! 
*************************************************/

module DMA (
    input CLK, // clock
    input BG, // bus granted signal
    input [4 * `WORD_SIZE - 1 : 0] edata, // data from external device
    input cmd, // command
    output BR, // bus request signal
    output WRITE, // memory write signal
    output [`WORD_SIZE - 1 : 0] addr, // d_memory address
    output [4 * `WORD_SIZE - 1 : 0] data, // data to write in d_memory
    output reg [1:0] offset, // offset for fetching data from external device
    output reg interrupt // dma_end signal
    );

    /* Implement your own logic */
    // fixed address = 0x01f4
    wire [`WORD_SIZE-1:0] fixedAddr;
    assign fixedAddr = `WORD_SIZE'h01f4;

    // DMA counter
    reg [3:0] dma_state; // 0 ~ 11 counter

    // address, data output register
    reg [`WORD_SIZE-1:0] dma_outputAddr;
    reg [`FETCH_SIZE - 1:0] dma_outputData;

    // assign address, data, WRITE according to BG
    assign addr = (BG)? dma_outputAddr : `WORD_SIZE'dz;
    assign data = (BG)? dma_outputData : `FETCH_SIZE'dz;
    assign WRITE = (BG && (dma_state == 4'd0 || dma_state == 4'd4 || dma_state == 4'd8))? 1'd1 : 1'dz;

    // BR assign
    assign BR = cmd;
    
    // update dma_state
    always @(posedge CLK) begin
        if (BG) begin
            dma_state <= (dma_state == 4'd11)? 4'd12 : dma_state + 4'd1;
        end
        else if (cmd) begin
            dma_state <= 4'd0;
        end
        else begin
            dma_state <= 4'd12;
        end
    end

    // update dma_end(interrupt), offset
    always @(posedge CLK) begin
        case(dma_state)
            4'd12 : begin
                // reset
                offset <= 2'd0;
                interrupt <= 1'd0;
            end
            4'd3 : offset <= 2'd1;
            4'd7 : offset <= 2'd2;
            4'd11 : begin
                offset <= 2'd0;
                interrupt <= 1'd1;
            end
        endcase
    end

    always @(*) begin
        // output dma data
        case (dma_state)
            4'd0 : begin
                dma_outputAddr <= fixedAddr;
                dma_outputData <= edata;
            end
            4'd4 : begin
                dma_outputAddr <= fixedAddr + `WORD_SIZE'd4;
                dma_outputData <= edata;
            end
            4'd8 : begin
                dma_outputAddr <= fixedAddr + `WORD_SIZE'd8;
                dma_outputData <= edata;
            end
        endcase
    end
endmodule


