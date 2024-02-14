package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public double climberPositionMeters = 0.0;
    public double climberAbsoluteVelocityMetersPerSec = 0.0;
    public double climberAppliedVolts = 0.0;
    public double[] climberCurrentAmps =
        new double[] {}; // Log motors individually, useful for failure analysis

    public double climberGoalPosition = 0.0;
    public double climberSetpointPosition = 0.0;
    public double climberSetpointVelocity = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimberIOInputs inputs) {}

  /** Sets the target of the climber * */
  public default void setPosition(double climberPositionRad) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  public default void setVelocity(double velocityMetersPerSec) {}

  public default void setHoming(boolean homingBool) {}

  /** Stop in open loop. */
  public default void stop() {
    setVoltage(0);
  }

  public void resetEncoder(final double position);

  public default void resetEncoder() {
    resetEncoder(0);
  }
}
