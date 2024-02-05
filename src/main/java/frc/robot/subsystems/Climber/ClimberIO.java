package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class climberIOInputs {
    public double climberArmPosition = 0.0;
    public double climberAppliedVolts = 0.0;
    public double[] climberCurrentAmps = new double[] {};
  }
  /** Updates the set of loggable inputs. */
  public default void updateInputs(climberIOInputs inputs) {}

  /** Sets the hight of the arm, in radians. */
  public default void setPosition(double armPositionPosition, double ffVolts) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Stop in open loop. */
  public default void stop() {}

  /** Set position PID constants. */
  public default void configurePID(double kP, double kI, double kD) {}
}
