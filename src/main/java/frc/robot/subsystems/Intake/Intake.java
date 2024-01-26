package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private final IntakeIO io;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void IndexerIn() {
    io.SetVoltage(Constants.IndexerConstants.indexerMotorVoltage);
  }

  public void IndexerOut() {
    io.SetVoltage(-Constants.IndexerConstants.indexerMotorVoltage);
  }

  public void IndexerStop() {
    io.SetVoltage(0);
  }
}