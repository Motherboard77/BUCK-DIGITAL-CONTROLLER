// -----------------------------------------------------------------------------
// buck_plant.sv : switching-level behavioral model of a synchronous buck stage
//   (simulation only; SystemVerilog `real` arithmetic, fixed time step = clk)
//
//   States : inductor current il, capacitor voltage vc
//   Switch : HS / LS ideal switches with Rds(on); body-diode conduction (VF)
//            during dead time; DCM (il held at 0) if both off and il reaches 0;
//            LS may conduct negative current (forced CCM)
//   Output : vo = vc + ESR*(il - io), solved algebraically for a resistive load
//   Integr.: semi-implicit (symplectic) Euler -> stable for the LC pair at the
//            2.5 ns step used here (dt << sqrt(LC) = 15 us)
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
module buck_plant #(
  parameter real L    = 2.2e-6,
  parameter real RDCR = 10e-3,
  parameter real RDS  = 10e-3,     // per switch
  parameter real VF   = 0.6,       // body-diode drop
  parameter real C    = 220e-6,
  parameter real RESR = 3e-3,
  parameter real DT   = 2.5e-9     // seconds per clk step
) (
  input  logic clk,
  input  logic hs,
  input  logic ls,
  input  real  vin,
  input  real  rload,
  output real  il,
  output real  vo,
  output real  vsw
);
  real vc, v_drive, r_ser, il_n, io, vo_n;

  initial begin il = 0.0; vc = 0.0; vo = 0.0; vsw = 0.0; end

  always @(posedge clk) begin
    vo_n = (vc + RESR*il) / (1.0 + RESR/rload);

    if (hs) begin                                  // HS on (shoot-through treated as HS on; TB flags it)
      v_drive = vin;  r_ser = RDS + RDCR;
    end else if (ls) begin
      v_drive = 0.0;  r_ser = RDS + RDCR;
    end else if (il > 0.0) begin                   // dead time, LS body diode
      v_drive = -VF;  r_ser = RDCR;
    end else if (il < 0.0) begin                   // dead time, HS body diode
      v_drive = vin + VF; r_ser = RDCR;
    end else begin                                 // DCM idle: both off, il = 0
      v_drive = vo_n; r_ser = 0.0;
    end
    vsw = v_drive;

    il_n = il + (v_drive - il*r_ser - vo_n) * DT / L;
    if (!hs && !ls) begin                          // diode turns off at zero crossing
      if ((il > 0.0 && il_n < 0.0) || (il < 0.0 && il_n > 0.0)) il_n = 0.0;
    end

    io = vo_n / rload;
    vc = vc + (il_n - io) * DT / C;
    il = il_n;
    vo = (vc + RESR*il) / (1.0 + RESR/rload);
  end
endmodule
