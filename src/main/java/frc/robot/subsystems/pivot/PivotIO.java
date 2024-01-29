package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

  @AutoLog
  public static class PivotIOInputs {
    public double pivotAbsolutePositionRad = 0.0;
    public double pivotAbsoluteVelocityRadPerSec = 0.0;
    public double pivotAppliedVolts = 0.0;
    public double[] pivotCurrentAmps =
        new double[] {}; // Log motors individually, useful for failure analysis

    public double pivotGoalPosition = 0.0;
    public double pivotSetpointPosition = 0.0;
    public double pivotSetpointVelocity = 0.0;
    public boolean openLoopStatus = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(PivotIOInputs inputs) {}

  /** Sets the angle of the pivot, in radians. */
  public default void setPosition(double positionRad) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Stop in open loop. */
  public default void stop() {}
}
