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
    input CLK, 
    input BG,
    input [4 * `WORD_SIZE - 1 : 0] edata,
    input cmd,
    output reg BR = 1'd0, 
    output WRITE,
    output [`WORD_SIZE - 1 : 0] addr, 
    output [4 * `WORD_SIZE - 1 : 0] data,
    output reg [1:0] offset = 2'd0,
    output interrupt
    );

    /* Implement your own logic */
    // fixed address = 0x01f4
    wire [`WORD_SIZE-1:0] fixedAddr;
    assign fixedAddr = `WORD_SIZE'h01f4;

    // dmaState
    reg [3:0] dma_counter; // 0 ~ 11 counter
    reg dma_finish = 1'd0;


    reg [`WORD_SIZE-1:0] dma_outputAddr;
    reg [`FETCH_SIZE - 1:0] dma_outputData;
    assign addr = (BG)? dma_outputAddr : `WORD_SIZE'dz;
    assign data = (BG)? dma_outputData : `FETCH_SIZE'dz;

    assign interrupt = (dma_counter == 4'd11);
    assign WRITE = BG && (dma_counter == 4'd0 || dma_counter == 4'd4 || dma_counter == 4'd8);

    always @(posedge CLK) begin
        if (BG) begin
            dma_outputAddr <= fixedAddr;
            dma_counter <= (dma_counter == 4'd11)? 4'd0 : dma_counter + 4'd1;
        end
        else begin
            dma_outputAddr <= `WORD_SIZE'dz;
            dma_counter <= 4'd0;
        end
    end

    always @(posedge CLK) begin
        case(dma_counter)
            4'd0 : begin
                offset <= 2'd0;
                dma_finish <= 1'd0;
            end
            4'd3 : offset <= 2'd1;
            4'd7 : offset <= 2'd2;
            4'd11 : begin
                offset <= 2'd0;
                dma_finish <= 1'd1;
            end
        endcase
    end

    always @(*) begin
        BR <= (dma_finish)? 1'd0 : cmd;
        case (dma_counter)
            4'd0 : dma_outputData <= (BG)? edata : `FETCH_SIZE'dz;
            4'd4 : dma_outputData <= edata;
            4'd8 : dma_outputData <= edata;
        endcase
    end
            




    
endmodule


