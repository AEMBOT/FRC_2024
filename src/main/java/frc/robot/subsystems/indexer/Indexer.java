package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
  public enum indexerStates{
    INDEX_ON_INTAKE_ON,
    INDEX_ON_INTAKE_OFF,
    INDEX_OFF_INTAKE_ON,
    INDEX_OFF_INTAKE_OFF,
  }
  indexerStates state = indexerStates.INDEX_OFF_INTAKE_ON;
  public Indexer(IndexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    switch (state) {
      case INDEX_ON_INTAKE_ON:
        indexerIn();
        intakeIn();
        if(inputs.shooterBeamBreakState){
          state = indexerStates.INDEX_OFF_INTAKE_OFF;
        }
        break;
      case INDEX_ON_INTAKE_OFF:
        indexerIn();
        intakeStop();
        if(!inputs.shooterBeamBreakState){
          state = indexerStates.INDEX_OFF_INTAKE_ON;
        }
        break;
      case INDEX_OFF_INTAKE_ON:
        indexerStop();
        intakeIn();
        if(inputs.intakeBeamBreakState){
          state = indexerStates.INDEX_ON_INTAKE_ON;
        }
        break;
      case INDEX_OFF_INTAKE_OFF:
        indexerStop();
        intakeStop();
        break;   
      default:
        indexerStop();
        intakeStop();
        break;
    }

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

  public void intake() {io.intakeRun();}

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

  public Command intakeCommand() {
    return runOnce(this::intake);
  }
}
