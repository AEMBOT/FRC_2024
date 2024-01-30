package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  public void IndexerIn() {
    io.setShooterIndexer(IndexerIO.IndexerIOInputs.MotorState.IN);
  }

  public void IndexerOut() {
    io.setShooterIndexer(IndexerIO.IndexerIOInputs.MotorState.OUT);
  }

  public void IndexerStop() {
    io.setShooterIndexer(IndexerIO.IndexerIOInputs.MotorState.OFF);
  }

  public void IntakeIn() {
    io.setIntakeIndexer(IndexerIO.IndexerIOInputs.MotorState.IN);
  }

  public void IntakeOut() {
    io.setIntakeIndexer(IndexerIO.IndexerIOInputs.MotorState.OUT);
  }

  public void IntakeStop() {
    io.setIntakeIndexer(IndexerIO.IndexerIOInputs.MotorState.OFF);
  }
}
