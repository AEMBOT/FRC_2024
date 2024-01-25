package frc.robot.subsystems.Indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  public void IndexerIn() {
    io.setVoltage(Constants.IndexerConstants.indexerMotorVoltage);
  }

  public void IndexerOut() {
    io.setVoltage(-Constants.IndexerConstants.indexerMotorVoltage);
  }

  public void IndexerStop() {
    io.setVoltage(0);
  }
}
