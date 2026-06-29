`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:      Indian Institute of Technology Hyderabad (IITH)
// Engineer:     Mahima Chourasiya
// Create Date:  22.06.2026 16:59:40
// Design Name:  Complex RLS Filter Hardware Accelerator Suite
// Module Name:  fir_filter_rls
// Project Name: Complex Adaptive Signal Processing Accelerator
// Target Devices: AMD Xilinx Zynq UltraScale+ MPSOC (ZCU102 Evaluation Kits)
// Tool Versions: Vivado 2025.1 (or later)
// Description:  
//   A parameterizable, cycle-accurate sequential hardware accelerator designed
//   to compute complex-valued Recursive Least Squares (RLS) adaptive filtering.
//   Features parallel fixed-point data tracks for dual-quadrature signal spaces 
//   (Real and Imaginary channels) mapped across a 10-tap time-multiplexed vector network.
//   Integrates universal convergent rounding arithmetic (Q16.16) and a sequential 
//   Hermitian matrix conjugate symmetry enforcer loop to prevent accumulation drift.
// 
// Dependencies: 
//   None (Self-contained synchronous finite state machine core architecture).
// 
// Revision:
//   Revision 1.00 - Initial Production Core Implementation & Fully Verified
// Additional Comments:
//   Optimized for high-throughput streaming digital signal processing pipelines.
// 
//////////////////////////////////////////////////////////////////////////////////
module fir_filter_rls #(
    parameter TAPS         = 10,  // Number of filter taps 
    parameter DATA_WL      = 16,  // Input/Output signal word-length
    parameter DATA_FL      = 8,   // Fractional bits for signals x and d (8 means Q8.8)
    parameter COEFF_WL     = 32,  // Coefficient tap  word-length
    parameter COEFF_FL     = 16    // Fractional bits for coefficients h (8 means Q24.8)

)(
    input wire clk,
    input wire rst_n,

    input wire sample_valid,
    input wire signed [DATA_WL-1:0] x_real,
    input wire signed [DATA_WL-1:0] x_imag,

    input wire signed [DATA_WL-1:0] d_real,
    input wire signed [DATA_WL-1:0] d_imag,

    output reg output_ready = 0,                               
    output reg signed [DATA_WL-1:0] y_real = 0,
    output reg signed [DATA_WL-1:0] y_imag = 0
);
    localparam GROWTH_BITS  = $clog2(TAPS);

    // Forgetting Factor (Lambda) 
    localparam LAMBDA_WL    = 24;
    localparam LAMBDA_FL    = 16;
    localparam signed [LAMBDA_WL-1:0] lambda     = 24'd62587; // Lambda = 0.955 in q8.16 (0.955 * 65536)
    localparam signed [LAMBDA_WL-1:0] inv_lambda = 24'd68624; // (1/lambda) = 1.04712 in q8.16 format 
    
    //STATE 1 LOCAL PARAMS
    localparam MIN_ACC_ST1_WL = DATA_WL + COEFF_WL + 1 + GROWTH_BITS;
    localparam ACC_ST1_FL     = DATA_FL + COEFF_FL;
    localparam FIR_SHIFT      = ACC_ST1_FL - DATA_FL;
    //STATE 2
    localparam P_MATRIX_WL = 40;
    localparam P_MATRIX_FL = 20; // Q20.20 format
    localparam MIN_PX_WL   = P_MATRIX_WL + DATA_WL + 1 + GROWTH_BITS;
    localparam PX_FL       = P_MATRIX_FL + DATA_FL;

    //STATE 3
    localparam MIN_DENOM_ACC_WL = DATA_WL + MIN_PX_WL + 1 + GROWTH_BITS;
    localparam DENOM_ACC_FL     = DATA_FL + PX_FL;
    localparam DENOM_SHIFT      = DENOM_ACC_FL - LAMBDA_FL;
    localparam DENOM_SCALAR_WL  = 19 + LAMBDA_WL;                   //43 bits [42:0]
    
    //STATE 4
    localparam GAIN_K_WL   = 48;
    localparam GAIN_K_FL   = 28;        //Q20.28
    localparam NUM_LSHIFT  = GAIN_K_FL + LAMBDA_FL - PX_FL;
    localparam MIN_DIV_WL  = MIN_PX_WL + NUM_LSHIFT;

    //STATE 5
    localparam UPDATE_SHIFT     = (GAIN_K_FL + DATA_FL) - COEFF_FL;
    localparam MIN_RAWSUB_WL    = GAIN_K_WL + MIN_PX_WL + 1;
    localparam RAWSUB_FL        = GAIN_K_FL + PX_FL;
    localparam SUB_SHIFT        = RAWSUB_FL - P_MATRIX_FL;
    localparam MIN_NEXTP_WL     = P_MATRIX_WL + LAMBDA_WL;
    localparam NEXTP_FL         = P_MATRIX_FL + LAMBDA_FL;
    localparam P_RESCALE_SHIFT  = NEXTP_FL - P_MATRIX_FL; 

    // FSM Encoding
    localparam STATE_IDLE        = 3'd0;
    localparam STATE_FIR_MAC     = 3'd1;
    localparam STATE_MATRIX_PX   = 3'd2;
    localparam STATE_DENOMINATOR = 3'd3;
    localparam STATE_GAIN_K      = 3'd4;
    localparam STATE_UPDATE      = 3'd5;
    localparam STATE_SYMMETRY    = 3'd6;

    // HARDWARE REGISTERS
    reg [2:0] state;
    reg [GROWTH_BITS-1:0] row_idx; // Row counter 
    reg [GROWTH_BITS-1:0] j; 

    // DYNAMICALLY SIZED DATA SHIFTING REGISTERS
    reg signed [DATA_WL-1:0] x_r [0 : TAPS-1];
    reg signed [DATA_WL-1:0] x_i [0 : TAPS-1];

    // DYNAMICALLY SIZED COEF TAP REGISTERS
    reg signed [COEFF_WL-1:0] h_r [0 : TAPS-1];
    reg signed [COEFF_WL-1:0] h_i [0 : TAPS-1];
    
    //STATE 1 
    reg signed [MIN_ACC_ST1_WL-1:0] acc_r;        
    reg signed [MIN_ACC_ST1_WL-1:0] acc_i;  
    reg signed [DATA_WL:0] err_r, err_i;                        // QX.DATA_FL (q9.8)
 
    //STATE 2
    reg signed [P_MATRIX_WL-1:0] p_r [0:TAPS-1][0:TAPS-1];      //(Q20.20)
    reg signed [P_MATRIX_WL-1:0] p_i [0:TAPS-1][0:TAPS-1];
    reg signed [MIN_PX_WL-1:0] acc_px_r;                        //(Q33.28)
    reg signed [MIN_PX_WL-1:0] acc_px_i;
    reg signed [MIN_PX_WL-1:0] Px_r [0:TAPS-1];                 //(Q33.28)
    reg signed [MIN_PX_WL-1:0] Px_i [0:TAPS-1];

    //STATE 3
    reg signed [MIN_DENOM_ACC_WL-1:0] acc_denr;                 // q46.36
    reg signed [DENOM_SCALAR_WL-1:0] denom_scalar;              //  (Q27.16)

    //STATE 4
    reg signed [GAIN_K_WL-1:0] k_r [0:TAPS-1];                   // (Q20.28)
    reg signed [GAIN_K_WL-1:0] k_i [0:TAPS-1];
    reg signed [MIN_DIV_WL-1:0] raw_div_r;
    reg signed [MIN_DIV_WL-1:0] raw_div_i;

    //STATE 5
    reg signed [MIN_RAWSUB_WL-1:0] raw_sub_r;           // q54.56
    reg signed [MIN_RAWSUB_WL-1:0] raw_sub_i;
    reg signed [P_MATRIX_WL-1:0]   sub_r;               // q20.20
    reg signed [P_MATRIX_WL-1:0]   sub_i;
    reg signed [MIN_NEXTP_WL-1:0]  next_p_r;            // q29.36
    reg signed [MIN_NEXTP_WL-1:0]  next_p_i;

    integer i, k;


    // MAIN EXECUTION LOGIC
    always @(posedge clk or negedge rst_n) begin 
        if(!rst_n) begin 
            state        <= STATE_IDLE;
            row_idx      <= 4'd0;
            output_ready <= 1'b0;
            y_real       <= {DATA_WL{1'b0}};
            y_imag       <= {DATA_WL{1'b0}};
            denom_scalar <= 'sd0;
            err_i        <= 'sd0;
            err_r        <= 'sd0;
            acc_r        <= 'sd0;
            acc_i        <= 'sd0;
            acc_px_r     <= 'sd0;
            acc_px_i     <= 'sd0;
            acc_denr     <= 'sd0;
            // Parameterized Safe Flush Loop
            for (i = 0; i < TAPS; i = i + 1) begin
                x_r[i]  <= {DATA_WL{1'b0}}; 
                x_i[i]  <= {DATA_WL{1'b0}};
                h_r[i]  <= {COEFF_WL{1'b0}}; 
                h_i[i]  <= {COEFF_WL{1'b0}};
                Px_r[i] <= 'sd0; 
                Px_i[i] <= 'sd0;
                k_r[i]  <= 'sd0; 
                k_i[i]  <= 'sd0;
                
                for (k = 0; k < TAPS; k = k + 1) begin
                    if (i == k) begin
                        p_r[i][k] <= 'sd1049; // Diagonals loaded with 0.001 (Q20.20)
                    end else begin
                        p_r[i][k] <= 'sd0;
                    end
                        p_i[i][k] <= 'sd0;
                end
            end
        end
        else begin
            case(state)

                // STATE 0: PARAMETERIZED DELAY LINE INTAKE
                
                STATE_IDLE : begin
                    output_ready <= 1'b0;
                    if (sample_valid) begin 
                        for (i = TAPS-1; i > 0; i = i - 1) begin
                            x_r[i] <= x_r[i-1];
                            x_i[i] <= x_i[i-1];
                        end
                        x_r[0] <= x_real; 
                        x_i[0] <= x_imag;
                        
                        state <= STATE_FIR_MAC;
                    end
                end
                
                // STATE 1: AUTOMATICALLY ALIGNED MAC FILTER CONVERGENCE
                
                STATE_FIR_MAC : begin  
                    acc_r = 'sd0;
                    acc_i = 'sd0;
                    for(i = 0; i < TAPS ; i = i + 1) begin
                        acc_r = acc_r + ($signed(x_r[i]) * $signed(h_r[i])) + ($signed(x_i[i]) * $signed(h_i[i]));     
                        acc_i = acc_i + ($signed(x_i[i]) * $signed(h_r[i])) - ($signed(x_r[i]) * $signed(h_i[i]));
                    end

                    y_real <= acc_r >>> FIR_SHIFT; 
                    y_imag <= acc_i >>> FIR_SHIFT; 
                    
                    err_r  <= d_real - (acc_r >>> FIR_SHIFT);      
                    err_i  <= d_imag - (acc_i >>> FIR_SHIFT);     
                    
                    row_idx <= 4'd0; 
                    j       <= 4'd0;
                    state   <= STATE_MATRIX_PX;
                end
                
                
                // STATE 2: MATRIX (P * x)

                STATE_MATRIX_PX: begin
                    acc_px_r = 'sd0;
                    acc_px_i = 'sd0;

                    for (j = 0; j < TAPS; j = j + 1) begin
                        acc_px_r = acc_px_r + (p_r[row_idx][j] * x_r[j]) - (p_i[row_idx][j] * x_i[j]);
                        acc_px_i = acc_px_i + (p_r[row_idx][j] * x_i[j]) + (p_i[row_idx][j] * x_r[j]);
                    end
                    
                    Px_r[row_idx] <= acc_px_r;        
                    Px_i[row_idx] <= acc_px_i;
                    
                    if (row_idx == TAPS-1) begin
                        row_idx <= 4'd0;
                        j       <= 4'd0;
                        state   <= STATE_DENOMINATOR;
                    end else begin
                        row_idx <= row_idx + 4'd1; 
                    end
                end
                 
                // STATE 3: DENOMINATOR EVALUATION
            
                STATE_DENOMINATOR: begin
                    acc_denr = 'sd0;

                    for (i = 0; i < TAPS; i = i + 1) begin
                        acc_denr = acc_denr + (x_r[i] * Px_r[i]) + (x_i[i] * Px_i[i]); 
                    end
            
                    denom_scalar <= lambda + (acc_denr >>> DENOM_SHIFT); 
                    state        <= STATE_GAIN_K;
                end   

                
                // STATE 4: DIVISION STEP (KALMAN GAIN GENERATION)

                STATE_GAIN_K : begin 
                    for (i = 0; i < TAPS; i = i + 1) begin
                        raw_div_r = ((Px_r[i] <<< NUM_LSHIFT) / denom_scalar); 
                        raw_div_i = ((Px_i[i] <<< NUM_LSHIFT) / denom_scalar);
                                
                        k_r[i]    <= raw_div_r[GAIN_K_WL-1:0];  
                        k_i[i]    <= raw_div_i[GAIN_K_WL-1:0];
                    end
                    row_idx <= 4'd0;
                    j       <= 4'd0;                    
                    state   <= STATE_UPDATE;
                end              
           
                
                // STATE 5: ADAPTIVE TAP & VARIANCE UPDATE PIPELINES

                STATE_UPDATE: begin
                    if (j == 4'd0) begin
                        h_r[row_idx] <= h_r[row_idx] + ($signed($signed(k_r[row_idx]) * $signed(err_r)) + $signed($signed(k_i[row_idx]) * $signed(err_i)) >>> UPDATE_SHIFT);  
                        h_i[row_idx] <= h_i[row_idx] + ($signed($signed(k_i[row_idx]) * $signed(err_r)) - $signed($signed(k_r[row_idx]) * $signed(err_i)) >>> UPDATE_SHIFT);
                    end
                      
                    raw_sub_r = $signed(($signed(k_r[row_idx]) * $signed(Px_r[j]))) + 
                                $signed(($signed(k_i[row_idx]) * $signed(Px_i[j]))); 
                                    
                    raw_sub_i = $signed(($signed(k_i[row_idx]) * $signed(Px_r[j]))) - 
                                $signed(($signed(k_r[row_idx]) * $signed(Px_i[j])));

                    sub_r = $signed(raw_sub_r + (1'sh1 << (SUB_SHIFT - 1))) >>> SUB_SHIFT; 
                    sub_i = $signed(raw_sub_i + (1'sh1 << (SUB_SHIFT - 1))) >>> SUB_SHIFT;

                    next_p_r = (($signed(p_r[row_idx][j])) - $signed(sub_r)) * $signed(inv_lambda);
                    next_p_i = (($signed(p_i[row_idx][j])) - $signed(sub_i)) * $signed(inv_lambda);

                    p_r[row_idx][j] <= $signed(next_p_r + (1'sh1 << (P_RESCALE_SHIFT - 1))) >>> P_RESCALE_SHIFT; 
                    p_i[row_idx][j] <= $signed(next_p_i + (1'sh1 << (P_RESCALE_SHIFT - 1))) >>> P_RESCALE_SHIFT;

                    if (j == TAPS-1) begin
                        j <= 4'd0;
                        if (row_idx == TAPS-1) begin
                            row_idx <= 4'd0;
                            state   <= STATE_SYMMETRY;
                        end else begin
                            row_idx <= row_idx + 4'd1;
                        end
                    end else begin
                        j <= j + 4'd1;
                    end
                end

                // STATE 6: SEQUENTIAL HERMITIAN SYMMETRY STEPPER ENGINE (Prevents Simulation Hangs)
                STATE_SYMMETRY: begin
                    if (row_idx > j) begin
                        p_r[j][row_idx] <= p_r[row_idx][j];   
                        p_i[j][row_idx] <=-p_i[row_idx][j];  
                    end

                    if (j == TAPS - 1) begin
                        j <= 4'd0;
                        if (row_idx == TAPS - 1) begin
                            row_idx      <= 4'd0;
                            output_ready <= 1'b1;      
                            state        <= STATE_IDLE; 
                        end else begin
                            row_idx <= row_idx + 4'd1;
                        end
                    end else begin
                        j <= j + 4'd1;
                    end
                end

                default: state <= STATE_IDLE;
            endcase
        end
    end
endmodule