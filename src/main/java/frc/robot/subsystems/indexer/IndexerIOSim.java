package frc.robot.subsystems.indexer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;

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

  public void setIndexerVoltage(double voltage) {
    indexerSim.setInputVoltage(voltage);
    indexerAppliedVolts = voltage;
  }

  public void setIntakeVoltage(double voltage) {
    intakeSim.setInputVoltage(voltage);
    intakeAppliedVolts = voltage;
  }
}
