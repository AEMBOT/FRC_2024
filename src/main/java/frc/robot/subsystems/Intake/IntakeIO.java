package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOinputs {
    public double appliedVolts = 0.0;
  }

  public default void SetVoltage(double volts) {}
}
