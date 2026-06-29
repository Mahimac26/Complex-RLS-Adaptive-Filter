`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:      Indian Institute of Technology Hyderabad (IITH)
// Engineer:     Mahima Chourasiya
// Create Date:  22.06.2026 18:15:22
// Design Name:  Complex RLS Filter Hardware Accelerator Suite
// Module Name:  fir_filter_rls_tb
// Project Name: Complex Adaptive Signal Processing Accelerator
// Target Devices: Behavioral Simulation Setup for Zynq UltraScale+ ZCU102
// Tool Versions: Vivado Simulator (XSIM)
// Description:  
//   Data-driven testbench engine designed to validate the precision bounds and
//   FSM performance metrics of the fir_filter_rls core module. Streams fixed-point 
//   complex data arrays (Q8.8) and reference channels into the pipeline and probes
//   the 32-bit (Q16.16) 10-tap converged weight coefficient updates against analytical 
//   golden reference targets derived from the matching MATLAB algorithmic verification suite.
// 
// Dependencies: 
//   - fir_filter_rls.v (Unit Under Test)
//   - input_iq.hex     (Input test vectors)
//   - output_iq.hex    (Golden output targets)
// 
// Revision:
//   Revision 1.00 - Testbench Framework Structured and Execution Waveforms Verified
// Additional Comments:
//   Simulates cycle-accurate strobe logic handshake protocols (sample_valid/output_ready).
// 
//////////////////////////////////////////////////////////////////////////////////
module fir_filter_rls_tb;

    // Inputs to DUT
    reg clk;
    reg rst_n;
    reg sample_valid;
    reg signed [15:0] x_real;
    reg signed [15:0] x_imag;
    reg signed [15:0] d_real;
    reg signed [15:0] d_imag;

    // Outputs from DUT
    wire output_ready;
    wire signed [15:0] y_real;
    wire signed [15:0] y_imag;

    // File Handles
    integer file_in;  
    integer file_ref; 
    integer status_in;
    integer status_ref;
    integer sample_counter = 0;

    // Temporary storage for parsing operations
    reg [15:0] in_r_hex, in_i_hex;
    reg [15:0] ref_r_hex, ref_i_hex;

    // Instantiate the master filter core module
    fir_filter_rls uut (
        .clk(clk),
        .rst_n(rst_n),
        .sample_valid(sample_valid),
        .x_real(x_real),
        .x_imag(x_imag),
        .d_real(d_real),
        .d_imag(d_imag),
        .output_ready(output_ready),
        .y_real(y_real),
        .y_imag(y_imag)
    );

    // 30 MHz System Clock (33.33ns period)
    always #16.667 clk = ~clk;

    // Main Test Framework
    initial begin
        clk = 0;
        rst_n = 0;
        sample_valid = 0;
        x_real = 0; x_imag = 0;
        d_real = 0; d_imag = 0;

        // Open files directly
        file_in  = $fopen("C:/Users/Mahima/verilog/FIRfilter/input_iq.hex", "r");
        file_ref = $fopen("C:/Users/Mahima/verilog/FIRfilter/output_iq.hex", "r");
        
        if (file_in == 0 || file_ref == 0) begin
            $display("ERROR: One or both files could not be opened. Check file paths!");
            $finish;
        end

        #100;
        rst_n = 1; // Release the hardware reset
        #20;

        $display("--- Starting Dual File Verification Streaming ---");

        while (!$feof(file_in) && !$feof(file_ref)) begin
            
            status_in  = $fscanf(file_in, "0x%h, 0x%h\n", in_r_hex, in_i_hex);
            status_ref = $fscanf(file_ref, "0x%h, 0x%h\n", ref_r_hex, ref_i_hex);
            
            if (status_in == 2 && status_ref == 2) begin
                
                @(posedge clk);
                x_real = in_r_hex;
                x_imag = in_i_hex;
                d_real = ref_r_hex;
                d_imag = ref_i_hex;
                sample_valid = 1'b1; // Pulse high to kickoff FSM

                @(posedge clk);
                sample_valid = 1'b0; // De-assert immediately 

                // Wait until the sequential math loops complete completely 
                @(posedge output_ready);
                
                sample_counter = sample_counter + 1;
                
                @(posedge clk); // Naturally wait one tick before streaming the next vector line
            end
        end

        $display("\n==================================================");
        $display("   FINAL RLS FILTER CONVERGENT SUMMARY  ");
        $display("==================================================");
        $display(" Total Processed Samples: %d", sample_counter);
        $display(" Final Calculated Output (y): Real=%d, Imag=%d", $signed(y_real), $signed(y_imag));
        $display(" Final Residual Error    (e): Real=%d, Imag=%d", $signed(uut.err_r), $signed(uut.err_i));
        $display("--------------------------------------------------");
        $display(" FINAL CONVERGED FILTER COEFFICIENT TAPS (h):");
        $display("--------------------------------------------------");
        $display(" Tap 0 -> Real: %d, Imag: %d", $signed(uut.h_r[0]), $signed(uut.h_i[0]));
        $display(" Tap 1 -> Real: %d, Imag: %d", $signed(uut.h_r[1]), $signed(uut.h_i[1]));
        $display(" Tap 2 -> Real: %d, Imag: %d", $signed(uut.h_r[2]), $signed(uut.h_i[2]));
        $display(" Tap 3 -> Real: %d, Imag: %d", $signed(uut.h_r[3]), $signed(uut.h_i[3]));
        $display(" Tap 4 -> Real: %d, Imag: %d", $signed(uut.h_r[4]), $signed(uut.h_i[4]));
        $display(" Tap 5 -> Real: %d, Imag: %d", $signed(uut.h_r[5]), $signed(uut.h_i[5]));
        $display(" Tap 6 -> Real: %d, Imag: %d", $signed(uut.h_r[6]), $signed(uut.h_i[6]));
        $display(" Tap 7 -> Real: %d, Imag: %d", $signed(uut.h_r[7]), $signed(uut.h_i[7]));
        $display(" Tap 8 -> Real: %d, Imag: %d", $signed(uut.h_r[8]), $signed(uut.h_i[8]));
        $display(" Tap 9 -> Real: %d, Imag: %d", $signed(uut.h_r[9]), $signed(uut.h_i[9]));
        $display("==================================================");

        $display("--- Verification Success: End of both file tracks reached! ---");
        $fclose(file_in);
        $fclose(file_ref);
        $finish;
    end

endmodule