package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class PivotIOSim implements PivotIO {
  private boolean openLoop = false;
  private double feedForward = 0.0;
  private double appliedVolts = 0.0;
  private DCMotorSim sim = new DCMotorSim(DCMotor.getNEO(2), 1, 1); // Numbers TODO
  private PIDController controller = new PIDController(0.0,0.0,0.0);
  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(PivotIOInputs inputs) {
    if (openLoop) {
      appliedVolts = feedForward;
    } else {
      appliedVolts =
              MathUtil.clamp(controller.calculate(sim.getAngularPositionRad()) + feedForward, -12.0, 12.0);
      sim.setInputVoltage(appliedVolts);
    }
    sim.setInputVoltage(appliedVolts);
    sim.update(0.2);
    inputs.pivotAbsolutePositionRad = sim.getAngularPositionRad();
    inputs.pivotAppliedVolts = appliedVolts;
    inputs.pivotCurrentAmps = new double[]{sim.getCurrentDrawAmps(), sim.getCurrentDrawAmps()};
    inputs.pivotAbsoluteVelocityRadPerSec = sim.getAngularVelocityRadPerSec();
  }

  /** Sets the angle of the pivot, in radians. */
  @Override
  public void setPosition(double positionRad, double ffVolts) {
    openLoop = false;
    controller.setSetpoint(positionRad);
    feedForward = ffVolts;
  }

  /** Run open loop at the specified voltage. */
  @Override
  public void setVoltage(double volts) {
    openLoop = true;
    feedForward = volts;
  }

  /** Stop in open loop. */
  @Override
  public void stop() {
    setVoltage(0.0);
  }

  /** Set position PID constants. */
  @Override
  public void configurePID(double kP, double kI, double kD) {
    controller.setPID(kP, kI, kD);
  }
}
