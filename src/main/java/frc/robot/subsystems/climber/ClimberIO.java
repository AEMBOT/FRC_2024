package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public double climberLeftPositionMeters = 0.0;
    public double climberRightPositionMeters = 0.0;
    public double climberLeftVelocityMetersPerSec = 0.0;
    public double climberRightVelocityMetersPerSec = 0.0;
    public double climberLeftAppliedVolts = 0.0;
    public double climberRightAppliedVolts = 0.0;
    public double[] climberCurrentAmps =
        new double[] {}; // Log motors individually, useful for failure analysis
    public double climberSetpointPosition = 0.0;

    public boolean openLoopStatus = true;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimberIOInputs inputs) {}

  /** Sets the target of the climber * */
  public default void setPosition(double climberPositionRad) {}

  public default void setPositionClimbing(double climberPositionRad) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  public default void setHoming(boolean homingBool) {}

  public default void resetEncoder(final double position) {}

  public default void resetEncoder() {
    resetEncoder(0);
  }

  public default boolean isCurrentLimited() {
    return false;
  }
}
