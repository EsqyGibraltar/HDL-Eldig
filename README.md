# design.sv
module safety_fsm_controller (
  input  logic clk,
  input  logic rst_n,

  // 6 sensors (binary)
  input  logic FTip,        // ForceTip: 1 = over-force
  input  logic PosEnc,      // PositionEncoder: 1 = position valid
  input  logic Bound,       // BoundaryDetector: 1 = inside safe boundary
  input  logic EStop,       // EmergencyStop: 1 = NOT pressed, 0 = pressed
  input  logic Surgeon,     // Operator enable
  input  logic VAnom,       // VideoAnomaly: 1 = anomaly detected

  // 6 actuators (binary)
  output logic Freeze,      // MotionFreeze
  output logic FLimit,      // ForceLimitEnforce
  output logic Alarm,
  output logic Lock,        // ConsoleLock
  output logic Retract,     // AutoRetract
  output logic LogOut       // EventLog (named LogOut to avoid keyword confusion)
);

  // -------------------------------
  // Composite predicates
  // -------------------------------
  logic SafeAll, WarningCond, EmergencyCond;

  always_comb begin
    SafeAll       = EStop & (~FTip) & Bound & (~VAnom) & Surgeon & PosEnc;
    WarningCond   = FTip | VAnom;
    EmergencyCond = (~EStop) | (~Bound);
  end

  // -------------------------------
  // FSM states (2-bit encoding)
  // S0=00 IDLE, S1=01 DISPENSE, S2=10 SAFE_HOLD, S3=11 EMERGENCY
  // -------------------------------
  typedef enum logic [1:0] {
    S0_IDLE      = 2'b00,
    S1_DISPENSE  = 2'b01,
    S2_SAFEHOLD  = 2'b10,
    S3_EMERGENCY = 2'b11
  } state_t;

  state_t state, state_next;

  // -------------------------------
  // Next-state logic (policy)
  // Emergency dominates
  // -------------------------------
  always_comb begin
    state_next = state;

    if (EmergencyCond) begin
      state_next = S3_EMERGENCY;
    end else begin
      unique case (state)
        S0_IDLE: begin
          if (SafeAll) state_next = S1_DISPENSE;
          else         state_next = S0_IDLE;
        end

        S1_DISPENSE: begin
          if (WarningCond) state_next = S2_SAFEHOLD;
          else if (SafeAll) state_next = S1_DISPENSE;
          else              state_next = S1_DISPENSE; // stay
        end

        S2_SAFEHOLD: begin
          if (SafeAll)      state_next = S0_IDLE;
          else if (WarningCond) state_next = S2_SAFEHOLD;
          else              state_next = S2_SAFEHOLD;
        end

        S3_EMERGENCY: begin
          if (SafeAll) state_next = S0_IDLE;
          else         state_next = S3_EMERGENCY;
        end

        default: state_next = S0_IDLE;
      endcase
    end
  end

  // -------------------------------
  // State register (DFF)
  // -------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) state <= S0_IDLE;
    else        state <= state_next;
  end

  // -------------------------------
  // Moore outputs (based on state only)
  // Freeze  = ~Q0 + Q1
  // FLimit  =  Q0 + Q1
  // Alarm   =  Q1
  // Lock    =  Q1 & Q0
  // Retract =  Q1
  // Log     =  Q0 + Q1
  // -------------------------------
  always_comb begin
    Freeze  = 1'b1;
    FLimit  = 1'b0;
    Alarm   = 1'b0;
    Lock    = 1'b0;
    Retract = 1'b0;
    LogOut  = 1'b0;

    unique case (state)
      S0_IDLE: begin
        Freeze  = 1'b1;
        FLimit  = 1'b0;
        Alarm   = 1'b0;
        Lock    = 1'b0;
        Retract = 1'b0;
        LogOut  = 1'b0;
      end

      S1_DISPENSE: begin
        Freeze  = 1'b0;
        FLimit  = 1'b1;
        Alarm   = 1'b0;
        Lock    = 1'b0;
        Retract = 1'b0;
        LogOut  = 1'b1;
      end

      S2_SAFEHOLD: begin
        Freeze  = 1'b1;
        FLimit  = 1'b1;
        Alarm   = 1'b1;
        Lock    = 1'b0;
        Retract = 1'b1;
        LogOut  = 1'b1;
      end

      S3_EMERGENCY: begin
        Freeze  = 1'b1;
        FLimit  = 1'b1;
        Alarm   = 1'b1;
        Lock    = 1'b1;
        Retract = 1'b1;
        LogOut  = 1'b1;
      end
    endcase
  end

  # testbench.sv
  module tb;

  logic clk, rst_n;

  // Sensors
  logic FTip, PosEnc, Bound, EStop, Surgeon, VAnom;

  // Actuators
  logic Freeze, FLimit, Alarm, Lock, Retract, LogOut;

  // DUT
  safety_fsm_controller dut (
    .clk(clk), .rst_n(rst_n),
    .FTip(FTip), .PosEnc(PosEnc), .Bound(Bound), .EStop(EStop),
    .Surgeon(Surgeon), .VAnom(VAnom),
    .Freeze(Freeze), .FLimit(FLimit), .Alarm(Alarm), .Lock(Lock),
    .Retract(Retract), .LogOut(LogOut)
  );

  // clock: 10 time units period
  initial clk = 0;
  always #5 clk = ~clk;

  task apply_inputs(
    input logic t_FTip,
    input logic t_PosEnc,
    input logic t_Bound,
    input logic t_EStop,
    input logic t_Surgeon,
    input logic t_VAnom
  );
  begin
    FTip    = t_FTip;
    PosEnc  = t_PosEnc;
    Bound   = t_Bound;
    EStop   = t_EStop;
    Surgeon = t_Surgeon;
    VAnom   = t_VAnom;
  end
  endtask

  initial begin
    // dump for EPWave
    $dumpfile("wave.vcd");
    $dumpvars(0, tb);

    // init
    apply_inputs(0,0,1,1,0,0);
    rst_n = 0;

    // reset for a few cycles
    repeat (2) @(posedge clk);
    rst_n = 1;

    // --------------------------
    // Scenario:
    // S0 (idle) -> safe -> S1
    // warning -> S2
    // emergency -> S3
    // recover safe -> S0 -> safe -> S1
    // --------------------------

    // 1) SafeAll becomes true (should go S0 -> S1)
    // SafeAll = EStop=1, FTip=0, Bound=1, VAnom=0, Surgeon=1, PosEnc=1
    @(posedge clk);
    apply_inputs(0,1,1,1,1,0);

    repeat (2) @(posedge clk);

    // 2) Warning: ForceTip=1 (should go S1 -> S2)
    @(posedge clk);
    apply_inputs(1,1,1,1,1,0);

    repeat (2) @(posedge clk);

    // 3) Emergency: EStop pressed (EStop=0) (should go to S3)
    @(posedge clk);
    apply_inputs(0,1,1,0,1,0);

    repeat (2) @(posedge clk);

    // 4) still emergency (stay)
    @(posedge clk);
    apply_inputs(0,1,1,0,1,0);

    repeat (1) @(posedge clk);

    // 5) Recovery: restore SafeAll (EStop=1 and all safe) (S3 -> S0)
    @(posedge clk);
    apply_inputs(0,1,1,1,1,0);

    repeat (2) @(posedge clk);

    // 6) stay safe (S0 -> S1)
    @(posedge clk);
    apply_inputs(0,1,1,1,1,0);

    repeat (3) @(posedge clk);

    $finish;
  end

endmodule


endmodule
