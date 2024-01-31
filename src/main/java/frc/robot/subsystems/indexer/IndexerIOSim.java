package frc.robot.subsystems.indexer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import frc.robot.Constants;

public class IndexerIOSim implements IndexerIO {

  private DIOSim intakeSensor = new DIOSim(0);
  private DIOSim indexerSensor = new DIOSim(0);
  private DCMotorSim intakeSim = new DCMotorSim(DCMotor.getNEO(2), 0, 0);
  private DCMotorSim indexerSim = new DCMotorSim(DCMotor.getNEO(2), 0, 0);
  private IndexerIOInputs.MotorState indexerMotorsState = IndexerIOInputs.MotorState.OFF;
  private IndexerIOInputs.MotorState intakeMotorsState = IndexerIOInputs.MotorState.OFF;
  private boolean indexing = false;
  private boolean intaking = false;
  public DigitalInput intakeBeamBreak = new DigitalInput(Constants.IntakeConstants.intakeBeamBrake);
  public DigitalInput shooterBeamBreak =
      new DigitalInput(Constants.IndexerConstants.indexerBeamBrake);
  public boolean intakeBeamBreakState = false;
  public boolean shooterBeamBreakState = true;

  @Override
  public void updateInputs(IndexerIOInputs inputs) {

    intakeSim.update(0.02);
    indexerSim.update(0.02);

    inputs.intakeBeamBreakState = !intakeSensor.getValue();
    inputs.shooterBeamBreakState = !indexerSensor.getValue();

    if (intakeBeamBreakState && intaking) {
      intakeMotorsState = IndexerIOInputs.MotorState.OFF;
      indexerRun();
      intaking = false;
    }
    if (shooterBeamBreakState && indexing) {
      indexerMotorsState = IndexerIOInputs.MotorState.OFF;
      indexing = false;
    }

    switch (intakeMotorsState) {
      case OFF:
        intakeSim.setInputVoltage(0);
        break;
      case IN:
        intakeSim.setInputVoltage(Constants.IntakeConstants.intakeMotorVoltage);
        break;
      case OUT:
        intakeSim.setInputVoltage(-Constants.IntakeConstants.intakeMotorVoltage);
    }

    switch (indexerMotorsState) {
      case OFF:
        indexerSim.setInputVoltage(0);
        break;
      case IN:
        indexerSim.setInputVoltage(Constants.IndexerConstants.indexerMotorVoltage);
        break;
      case OUT:
        indexerSim.setInputVoltage(-Constants.IndexerConstants.indexerMotorVoltage);
    }
  }

  @Override
  public void setShooterIndexer(IndexerIOInputs.MotorState state) {
    indexerMotorsState = state;
  }

  @Override
  public void setIntakeIndexer(IndexerIOInputs.MotorState state) {
    intakeMotorsState = state;
  }

  @Override
  public void intakeRun() {
    intaking = true;
    setShooterIndexer(IndexerIOInputs.MotorState.IN);
  }

  @Override
  public void indexerRun() {
    indexing = true;
    setIntakeIndexer(IndexerIOInputs.MotorState.IN);
  }
}
