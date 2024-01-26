package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private final IntakeIO io;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void IntakeIn() {
    io.SetVoltage(Constants.IntakeConstants.intakeMotorVoltage);
  }

  public void IntakeOut() {
    io.SetVoltage(-Constants.IntakeConstants.intakeMotorVoltage);
  }

  public void IntakeStop() {
    io.SetVoltage(0);
  }
}