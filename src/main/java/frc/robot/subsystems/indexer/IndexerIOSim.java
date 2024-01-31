package frc.robot.subsystems.indexer;

import static frc.robot.Constants.IndexerConstants.indexerMotorVoltage;
import static frc.robot.Constants.IntakeConstants.intakeMotorVoltage;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import frc.robot.subsystems.indexer.IndexerIO.IndexerIOInputs.MotorState;

public class IndexerIOSim implements IndexerIO {
  private final DIOSim intakeBeamBreak = new DIOSim(0);
  private final DIOSim indexerBeamBreak = new DIOSim(1);
  private final DCMotorSim intakeSim = new DCMotorSim(DCMotor.getNEO(2), 1, 0.01);
  private final DCMotorSim indexerSim = new DCMotorSim(DCMotor.getNEO(2), 1, 0.01);

  private double intakeAppliedVolts = 0.0;
  private double indexerAppliedVolts = 0.0;

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    intakeSim.update(0.02);
    indexerSim.update(0.02);

    inputs.intakeBeamBreakState = !intakeBeamBreak.getValue();
    inputs.shooterBeamBreakState = !indexerBeamBreak.getValue();
    inputs.intakeIndexerAppliedVolts = intakeAppliedVolts;
    inputs.shooterIndexerAppliedVolts = indexerAppliedVolts;
    inputs.intakeIndexerCurrentAmps = new double[] {intakeSim.getCurrentDrawAmps()};
    inputs.shooterIndexerCurrentAmps = new double[] {indexerSim.getCurrentDrawAmps()};
  }

  @Override
  public void setShooterIndexer(MotorState state) {
    switch (state) {
      case OFF:
        indexerSim.setInputVoltage(0);
        indexerAppliedVolts = 0;
        break;
      case IN:
        indexerSim.setInputVoltage(indexerMotorVoltage);
        indexerAppliedVolts = indexerMotorVoltage;
        break;
      case OUT:
        indexerSim.setInputVoltage(-indexerMotorVoltage);
        indexerAppliedVolts = -indexerMotorVoltage;
    }
  }

  @Override
  public void setIntakeIndexer(MotorState state) {
    switch (state) {
      case OFF:
        intakeSim.setInputVoltage(0);
        intakeAppliedVolts = 0;
        break;
      case IN:
        intakeSim.setInputVoltage(intakeMotorVoltage);
        intakeAppliedVolts = intakeMotorVoltage;
        break;
      case OUT:
        intakeSim.setInputVoltage(-intakeMotorVoltage);
        intakeAppliedVolts = -intakeMotorVoltage;
    }
  }
}
