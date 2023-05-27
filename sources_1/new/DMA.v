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
    output reg BR, 
    output WRITE,
    output [`WORD_SIZE - 1 : 0] addr, 
    output [4 * `WORD_SIZE - 1 : 0] data,
    output reg [1:0] offset,
    output reg interrupt
    );

    /* Implement your own logic */
    // fixed address = 0x01f4
    wire [`WORD_SIZE-1:0] fixedAddr;
    assign fixedAddr = `WORD_SIZE'h01f4;

    // DMA counter
    reg [3:0] dma_counter; // 0 ~ 11 counter


    reg [`WORD_SIZE-1:0] dma_outputAddr;
    reg [`FETCH_SIZE - 1:0] dma_outputData;
    assign addr = (BG)? dma_outputAddr : `WORD_SIZE'dz;
    assign data = (BG)? dma_outputData : `FETCH_SIZE'dz;

    assign WRITE = (BG && (dma_counter == 4'd0 || dma_counter == 4'd4 || dma_counter == 4'd8))? 1'd1 : 1'dz;

    always @(posedge CLK) begin
        if (BR && !BG) begin
            dma_counter <= 4'd0;
        end
        else begin
            if (BG) dma_counter <= dma_counter + 4'd1;
            else dma_counter <= 4'd12;
        end
    end

    always @(posedge CLK) begin
        case(dma_counter)
            4'd12 : begin
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
        // bus request
        if (interrupt || dma_counter == 4'd12 && !cmd) BR <= 1'd0;
        else if (cmd) BR <= 1'd1;
        else BR <= BR;

        // output dma data
        case (dma_counter)
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


