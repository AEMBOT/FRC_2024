package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double[] shooterAppliedVolts = new double[] {};
    public double[] shooterVelocityRPM = new double[] {};
    public double[] shooterCurrentAmps =
        new double[] {}; // Log motors individually, useful for failure analysis
    public boolean openLoopStatus = true;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Run open loop at the specified voltage. Primarily for characterization. */
  public default void setVoltage(double volts) {}

  /** Run closed loop at the specified velocity. */
  public default void setVelocity(double velocityRPM) {}

  public default void setVelocityClimberMode(double velocityRPM){}

  /** Stop in open loop. */
  public default void stop() {}
}
