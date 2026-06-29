# Multi-Channel Complex Recursive Least Squares (RLS) Adaptive FIR Filter Hardware Accelerator

## 📝 Overview
An advanced, dynamically parameterizable, cycle-accurate sequential hardware accelerator implemented in Verilog to perform complex-valued **Recursive Least Squares (RLS)** adaptive filtering. Optimized for high-performance deployment on devices like the AMD Xilinx Zynq UltraScale+ RFSoC architectures, this module solves the matrix-based optimization problem recursively over time to track complex parameters, update internal inverse covariance matrices, and minimize estimation error metrics across parallel fixed-point data pipelines driven by a synchronous Finite State Machine (FSM).

## Mathematical Formulation

Given a complex streaming primary input vector $\mathbf{x}(n)$, a complex target reference signal $d(n)$, and a parameterizable tap length $M$, the engine dynamically adjusts the complex weight vector $\mathbf{h}(n)$ using an internal high-resolution inverse covariance matrix $\mathbf{P}(n)$ to compute optimal system convergence.

### 1. Complex Transposed FIR Filter Processing

**Analytical Core Equation:**

$$\mathbf{y}(n) = \mathbf{h}^H(n-1)\mathbf{x}(n)$$

**Hardware Implementation Summation Loop Forms:**

$$\text{y}_{\text{real}}(n) = \sum_{i=0}^{M-1} \left( x_{real}[i] \cdot h_{real}[i] + x_{imag}[i] \cdot h_{imag}[i] \right)$$

$$\text{y}_{\text{imag}}(n) = \sum_{i=0}^{M-1} \left( x_{imag}[i] \cdot h_{real}[i] - x_{real}[i] \cdot h_{imag}[i] \right)$$

### 2. Complex Error Residual Tracking

**Analytical Core Equation:**

$$e(n) = d(n) - y(n)$$

**Hardware Implementation Summation Loop Forms:**

$$\text{err}_{real}(n) = d_{real}(n) - \text{Re}\{y(n)\}$$

$$\text{err}_{imag}(n) = d_{imag}(n) - \text{Im}\{y(n)\}$$

### 3. Covariance Matrix Projection Vector Calculation

**Analytical Core Equation:**

$$\mathbf{p}(n) = \mathbf{P}(n-1)\mathbf{x}(n)$$

**Hardware Implementation Summation Loop Forms:**

$$\text{Px}_{\text{real}}[i] = \sum_{j=0}^{M-1} \left( P_{\text{real}}[i][j] \cdot x_{\text{real}}[j] - P_{\text{imag}}[i][j] \cdot x_{\text{imag}}[j] \right)$$

$$\text{Px}_{\text{imag}}[i] = \sum_{j=0}^{M-1} \left( P_{\text{real}}[i][j] \cdot x_{\text{imag}}[j] + P_{\text{imag}}[i][j] \cdot x_{\text{real}}[j] \right)$$
### 4. Dynamic Scalar Denominator Energy Evaluation

**Analytical Core Equation:**

$$d(n) = \lambda + \mathbf{x}^H(n)\mathbf{P}(n-1)\mathbf{x}(n)$$

**Hardware Implementation Summation Loop Forms:**

$$\text{denomscalar} = \lambda + \sum_{i=0}^{M-1} \left( x_{real}[i] \cdot \text{Px}_{real}[i] + x_{imag}[i] \cdot \text{Px}_{imag}[i] \right)$$

> 💡 **Why only the Real Part is calculated for `denom_scalar`:**
> Analytically, the denominator calculation contains a complex quadratic form expressed as:
> $$\mathbf{x}^H(n)\mathbf{P}(n-1)\mathbf{x}(n)$$
> Because the inverse covariance matrix $\mathbf{P}$ is a Hermitian positive-definite matrix ($P_{j,i} = P_{i,j}^*$), any quadratic product of this form mathematically forces all imaginary components to perfectly cancel out. The resulting scalar is guaranteed to be a pure real number ($\in \mathbb{R}$). To maximize hardware efficiency and minimize FPGA resource utilization, the hardware completely omits imaginary calculation tracks for this step, saving significant DSP multiplier and adder blocks.
### 5. Multi-Tap Kalman Gain Vector Generation

**Analytical Core Equation:**

$$\mathbf{k}(n) = \frac{\mathbf{P}(n-1)\mathbf{x}(n)}{\lambda + \mathbf{x}^H(n)\mathbf{P}(n-1)\mathbf{x}(n)}$$

**Hardware Implementation Summation Loop Forms:**

Because the denominator energy value is a pure real scalar, the hardware divides both the real and imaginary projection paths independently inside the pipeline to generate the complex gain components:

$$\mathbf{k}_{real}[i] = \frac{\text{Px}_{real}[i] \ll \text{NUMLSHIFT}}{\text{denomscalar}}$$

$$\mathbf{k}_{imag}[i] = \frac{\text{Px}_{imag}[i] \ll \text{NUMLSHIFT}}{\text{denomscalar}}$$
### 6. Parallel Weight Update and Matrix Evolution

**Analytical Core Equation:**

$$\mathbf{h}(n) = \mathbf{h}(n-1) + \mathbf{k}(n)e^*(n)$$
$$\mathbf{P}(n) = \frac{1}{\lambda} \left[ \mathbf{P}(n-1) - \mathbf{k}(n)\mathbf{x}^H(n)\mathbf{P}(n-1) \right]$$

**Hardware Implementation Summation Loop Forms:**

$$\mathbf{h}_{\text{real}}[i] = h_{\text{real}}[i] + \left( k_{\text{real}}[i] \cdot \text{err}_{\text{real}} + k_{\text{imag}}[i] \cdot \text{err}_{\text{imag}} \right)$$

$$\mathbf{h}_{\text{imag}}[i] = h_{\text{imag}}[i] + \left( k_{\text{imag}}[i] \cdot \text{err}_{\text{real}} - k_{\text{real}}[i] \cdot \text{err}_{\text{imag}} \right)$$

$$\mathbf{P}_{\text{real}}[i][j] = \frac{1}{\lambda} \left( P_{\text{real}}[i][j] - \left( k_{\text{real}}[i] \cdot \text{Px}_{\text{real}}[j] + k_{\text{imag}}[i] \cdot \text{Px}_{\text{imag}}[j] \right) \right)$$

$$\mathbf{P}_{\text{imag}}[i][j] = \frac{1}{\lambda} \left( P_{\text{imag}}[i][j] - \left( k_{\text{imag}}[i] \cdot \text{Px}_{\text{real}}[j] - k_{\text{real}}[i] \cdot \text{Px}_{\text{imag}}[j] \right) \right)$$

---

> 💡 **Why the update equation uses $\mathbf{Px}^H$ instead of $\mathbf{x}^H\mathbf{P}$:**
> 
> The raw mathematical update rule for the RLS filter requires calculating the vector-matrix product $\mathbf{x}^H(n)\mathbf{P}(n-1)$. However, computing this directly in hardware would require setting up an entirely new sequential loop matrix-multiplier structure, which wastes massive amounts of FPGA logic fabric and registers.
> 
> To bypass this, the architecture leverages the algebraic properties of complex numbers and matrices:
> 1. Because the inverse covariance matrix $\mathbf{P}$ is a symmetric **Hermitian matrix**, it is equal to its own conjugate transpose: $\mathbf{P} = \mathbf{P}^H$.
> 2. Using the conjugate transpose identity $(AB)^H = B^H A^H$, we can rewrite the expression by pulling out the pre-calculated projection vector $\mathbf{Px}$ from State 3:
> $$\mathbf{x}^H(n)\mathbf{P}(n-1) = \mathbf{x}^H(n)\mathbf{P}^H(n-1) = \left( \mathbf{P}(n-1)\mathbf{x}(n) \right)^H = \mathbf{Px}^H$$
> 
> By substituting $\mathbf{x}^H\mathbf{P}$ with $\mathbf{Px}^H$, the state machine completely eliminates the need for an extra matrix-multiplication routine. The hardware simply reuses the exact internal array data values computed during the projection cycle (`STATE_MATRIX_PX`), replacing a full matrix computation loop with a single-cycle complex vector outer product subtraction loop.
## Key Features

* **Parameterized Precision Framework:** Dynamically structures internal hardware array bit-widths (`MIN_ACC_ST1_WL`, `MIN_PX_WL`, `MIN_DENOM_ACC_WL`) at compile-time based on `TAPS`, `DATA_WL`, and `COEFF_WL`.
* **Universal Convergent Rounding:** Integrates an automatic rounding bias infrastructure inside the MAC drop-stages to fully generalize operation across tight fractional boundaries like `Q16.16` and `Q8.8` without structural DC truncation offsets.
* **Conjugate Symmetry Maintenance:** Features an active sequential Hermitian stepper engine (`STATE_SYMMETRY`) that forces $P(j,i) = P^*(i,j)$, preventing fixed-point rounding drift and stabilizing execution tracking loops.
* **Race-Condition Immune Pipelines:** Uses precise blocking/non-blocking data isolation inside wide product matrices to ensure zero-cycle stale variable utilization during mathematical transitions.

---

## Architecture & FSM States

The core processes the RLS updates cycle-by-cycle using a deterministic 7-state hardware controller:

1. **`STATE_IDLE` (3'd0):** Monitors `sample_valid`; shifts raw complex samples into tap lines and wipes runtime accumulation registers.
2. **`STATE_FIR_MAC` (3'd1):** Executes the complex conjugate FIR inner product and captures bounded tracking error outputs with convergent rounding adjustments.
3. **`STATE_MATRIX_PX` (3'd2):** Multiplies the $M \times M$ inverse covariance matrix by the input data vector.
4. **`STATE_DENOMINATOR` (3'd3):** Pools energy scaling variables together and factors in the forgetting factor ($\lambda$).
5. **`STATE_GAIN_K` (3'd4):** Executes the division pipeline across all taps to evaluate the active Kalman Gain vector.
6. **`STATE_UPDATE` (3'd5):** Adjusts tap coefficients $\mathbf{h}$ and processes the wide outer product matrix updates $\mathbf{k}\mathbf{Px}^H$ via explicit signed casting blocks.
7. **`STATE_SYMMETRY` (3'd6):** Loops through the matrix columns sequentially to apply conjugate symmetry across the diagonals before pulsing `output_ready`.

---

## Module Interface (I/O Signal List)

| Signal Name | Direction | Width | Type | Description |
| :--- | :--- | :--- | :--- | :--- |
| `clk` | Input | `1` | wire | High-speed system clock |
| `rst_n` | Input | `1` | wire | Asynchronous system reset layer (Active Low) |
| `sample_valid` | Input | `1` | wire | Control strobe validating the presence of input samples |
| `x_real` | Input | `DATA_WL` | wire | Real coordinate stream of the input signal vector ($x_{real}$) |
| `x_imag` | Input | `DATA_WL` | wire | Imaginary coordinate stream of the input signal vector ($x_{imag}$) |
| `d_real` | Input | `DATA_WL` | wire | Real reference tracking channel destination value ($d_{real}$) |
| `d_imag` | Input | `DATA_WL` | wire | Imaginary reference tracking channel destination value ($d_{imag}$) |
| `output_ready` | Output | `1` | reg | High-asserted logic loop handshake finish notification |
| `y_real` | Output | `DATA_WL` | reg | Scaled real filter output response matrix ($y_{real}$) |
| `y_imag` | Output | `DATA_WL` | reg | Scaled imaginary filter output response matrix ($y_{imag}$) |

---

## Verification & Testbench Results

### 1. Functional HDL Timing Waveform
The timing trace from `image_434596.png` captures the cycle-accurate execution of the complex RLS accelerator core. Upon asserting the `sample_valid` control strobe, the internal FSM cycles deterministically through its state loops—sequentially tracking the rows and columns via the index counters without any stall or hang states—and securely latches the processing array data inputs before pulsing `output_ready`.

![Vivado Simulation Waveform Trace](image_a00179.png)

### ⏱️ Hardware Execution Latency & Performance Verification

The total real-time processing latency of the complex RLS accelerator core is explicitly verified using cycle-accurate timing analysis within the Vivado simulator engine. 

#### 1. Fundamental Timing Parameters
* **Testbench Clock Configuration:** `always #16.667 clk = ~clk;`
* **Clock Half-Period ($t_{\text{half}}$):** $16.667\,\text{ns}$
* **Full Clock Period ($T_{\text{clk}}$):** $16.667\,\text{ns} \times 2 = 33.334\,\text{ns}$
* **Effective System Clock Frequency ($f_{\text{clk}}$):** $\approx 30\,\text{MHz}$

#### 2. Latency Calculation via Simulation Timestamps
The total processing execution window is measured directly from the hardware handshake control lines:
* **Start Time ($T_{\text{start}}$):** $0.150003\,\mu\text{s}$ *(When `sample_valid` is asserted high to latch the incoming sample frame)*
* **End Time ($T_{\text{end}}$):** $7.250145\,\mu\text{s}$ *(When `output_ready` is asserted high indicating full matrix update completion )*

$$\Delta T_{\text{processing}} = T_{\text{end}} - T_{\text{start}}$$
$$\Delta T_{\text{processing}} = 7.250145\,\mu\text{s} - 0.150003\,\mu\text{s} = \mathbf{7.100142\,\mu\text{s}}$$

To determine the exact, absolute number of clock cycles consumed during this active processing window:

$$\text{Total Clock Cycles} = \frac{\Delta T_{\text{processing}}}{T_{\text{clk}}} = \frac{7100.142\,\text{ns}}{33.334\,\text{ns}} = \mathbf{213\text{ Clock Cycles}}$$

#### 3. Execution Summary Table

| Operational Metric | Value | Technical Context |
| :--- | :---: | :--- |
| **Absolute Latency Period** | **$7.100\,\mu\text{s}$** | Total time required to process a complex sample update. |
| **Exact Clock Cycle Count** | **213 Cycles** | Full data-path duration including internal BRAM reads and pipelined DSP math. |
| **Inter-Sample Turnaround** | **2 Cycles ($0.066\,\mu\text{s}$)** | Overhead before the testbench streams the next subsequent sample frame. |

> **Design Note:** This metric confirms that the sequential, time-multiplexed architecture successfully fulfills hard real-time execution constraints for high-throughput streaming digital signal processing pipelines on the AMD Xilinx Zynq UltraScale+ platform. It completes all recursive matrix transformations long before standard communications sample windows close, while maintaining a highly optimized, minimized physical resource footprint on the FPGA fabric.
### 2. Adaptive 10-Tap Coefficient Correlation Matrix ($Q16.16$)

The table below contrasts the final converged Real (`h_r`) and Imaginary (`h_i`) hardware fixed-point filter tap weights running under a **32-bit width / 16-bit fractional format** directly against the floating-point reference vectors derived in the MATLAB environment.

| Tap Index | MATLAB Real Target | Verilog Real Fixed-Point | MATLAB Imag Target | Verilog Imag Fixed-Point | Status |
| :---: | :--- | :--- | :--- | :--- | :---: |
| **`h[0]`** | `0.099938` | `0.09989929` | `0.049981` | `0.04896240` | **PASSED** |
| **`h[1]`** | `-0.050000` | `-0.04997253` | `0.020018` | `0.01994324` | **PASSED** |
| **`h[2]`** | `0.080002` | `0.07995605` | `-0.029988` | `-0.03009033` | **PASSED** |
| **`h[3]`** | `0.120005` | `0.11993408` | `0.040005` | `0.03991699` | **PASSED** |
| **`h[4]`** | `-0.030002` | `-0.03007507` | `-0.059962` | `-0.06008911` | **PASSED** |
| **`h[5]`** | `0.069966` | `0.06991577` | `0.009944` | `0.00981140` | **PASSED** |
| **`h[6]`** | `0.019987` | `0.01986694` | `-0.019997` | `-0.02003479` | **PASSED** |
| **`h[7]`** | `-0.039965` | `-0.04013061` | `0.030022` | `0.02996826` | **PASSED** |
| **`h[8]`** | `0.059970` | `0.05987549` | `-0.010027` | `-0.01002502` | **PASSED** |
| **`h[9]`** | `0.089980` | `0.08987427` | `0.020009` | `0.01994324` | **PASSED** |

### 3. Software vs. Hardware Fixed-Point Design Calibration

| Metric Reference Parameter | Functional Specification Value | Fixed-Point Notation Format | Description |
| :--- | :--- | :--- | :--- |
| **`TAPS`** | `10` | *Integer Scalar* | Total number of adaptive filter taps |
| **`DATA_WL` / `DATA_FL`** | `16` / `8` | `Q8.8` | Word length / Fraction for signals $x$ and $d$ |
| **`COEFF_WL` / `COEFF_FL`** | `32` / `16` | `Q16.16` | High-density coefficient weight configuration |
| **`GAIN_K_FL`** | `28` | `Q20.28` | Internal Kalman gain calculation tracking format |
| **`P_MATRIX_FL`** | `20` | `Q20.20` | Inverse covariance resolution metrics |
---
---

## 🎯 Implementation Summary & Deployment

The Multi-Channel Complex RLS Adaptive FIR Filter Hardware Accelerator achieves full mathematical convergence matching floating-point MATLAB behaviors within an optimized, production-ready footprint. By leveraging structural identities like the Hermitian conjugate rewrite ($\mathbf{x}^H\mathbf{P} = \mathbf{Px}^H$) and time-multiplexing computationally intensive array matrix calculations over a sequential hardware loop framework, the architecture comfortably satisfies real-time execution bounds ($7.100\,\mu\text{s}$) while retaining peak resource efficiency.

### 🚀 Future Optimization Steps
* **AXI-Stream Interface Integration:** Wrapping the physical core inside a native AXI4-Stream IP block wrapper to streamline high-speed direct-memory-access (DMA) data pipelines on the AMD Xilinx Zynq UltraScale+ processing subsystem (PS) fabric.
* **Dual-Port RAM Interleaving:** Upgrading internal Block RAM structures to true dual-port layouts to fetch row matrix variables concurrently, targeting a reduction of processing latency down below 120 clock cycles.

---
*Developed as a high-performance Hardware Descriptive Language (HDL) design accelerator for adaptive signal processing and real-time noise cancellation applications on heterogeneous system-on-chip platforms.*
