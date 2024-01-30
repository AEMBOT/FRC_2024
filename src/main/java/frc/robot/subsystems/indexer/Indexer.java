package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  public void indexerIn() {
    io.setShooterIndexer(IndexerIO.IndexerIOInputs.MotorState.IN);
  }

  public void indexerOut() {
    io.setShooterIndexer(IndexerIO.IndexerIOInputs.MotorState.OUT);
  }

  public void indexerStop() {
    io.setShooterIndexer(IndexerIO.IndexerIOInputs.MotorState.OFF);
  }

  public void intakeIn() {
    io.setIntakeIndexer(IndexerIO.IndexerIOInputs.MotorState.IN);
  }

  public void intakeOut() {
    io.setIntakeIndexer(IndexerIO.IndexerIOInputs.MotorState.OUT);
  }

  public void intakeStop() {
    io.setIntakeIndexer(IndexerIO.IndexerIOInputs.MotorState.OFF);
  }

  // Commands

  public Command intakeInCommand() {
    return runOnce(this::intakeIn);
  }

  public Command intakeOutCommand() {
    return runOnce(this::intakeOut);
  }

  public Command intakeStopCommand() {
    return runOnce(this::intakeStop);
  }

  public Command indexerInCommand() {
    return runOnce(this::indexerIn);
  }

  public Command indexerOutCommand() {
    return runOnce(this::indexerOut);
  }

  public Command indexerStopCommand() {
    return runOnce(this::indexerStop);
  }
}
