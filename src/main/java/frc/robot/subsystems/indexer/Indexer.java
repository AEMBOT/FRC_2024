package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  public void IndexerIn() {
    io.runShooterIndexer(true);
  }

  public void IndexerOut() {
    io.reverseShooterIndexer();
  }

  public void IndexerStop() {
    io.reverseIntakeIndexer();
  }
}
