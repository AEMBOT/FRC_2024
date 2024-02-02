package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public double climberPositionRad = 0.0;
    public double climberAbsoluteVelocityRadPerSec = 0.0;
    public double climberAppliedVolts = 0.0;
    public double[] climberCurrentAmps =
        new double[] {}; // Log motors individually, useful for failure analysis
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimberIOInputs inputs) {}

  /** Sets the angle of the pivot, in radians. */
  public default void setPosition(double climberPositionRad, double ffVolts) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Stop in open loop. */
  public default void stop() {}

  /** Set position PID constants. */
  public default void configurePID(double kP, double kI, double kD) {}
}
