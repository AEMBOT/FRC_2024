package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double shooterVelocityRadPerSec = 0.0;
    public double shooterAppliedVolts = 0.0;
    public double[] shooterCurrentAmps =
        new double[] {}; // Log motors individually, useful for failure analysis
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Run open loop at the specified voltage. Primarily for characterization. */
  public default void setVoltage(double volts) {}

  /** Run closed loop at the specified velocity. */
  public default void setVelocity(double velocityRadPerSec, double ffVolts) {}

  /** Stop in open loop. */
  public default void stop() {}

  /** Set velocity PID constants. */
  public default void configurePID(double kP, double kI, double kD) {}
}