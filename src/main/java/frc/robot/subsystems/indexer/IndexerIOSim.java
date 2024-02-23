package frc.robot.subsystems.indexer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class IndexerIOSim implements IndexerIO {
  private final DIOSim intakeBeamBreak = new DIOSim(0);
  private final DIOSim indexerBeamBreak = new DIOSim(1);
  private final DCMotorSim intakeSim = new DCMotorSim(DCMotor.getNEO(2), 1, 0.01);
  private final DCMotorSim indexerSim = new DCMotorSim(DCMotor.getNEO(2), 1, 0.01);

  private double intakeAppliedVolts = 0.0;
  private double indexerAppliedVolts = 0.0;

  public IndexerIOSim() {
    new Trigger(() -> intakeSim.getAngularVelocityRPM() > 1.0)
        .debounce(0.2)
        .onTrue(Commands.runOnce(() -> intakeBeamBreak.setValue(true)));
    new Trigger(() -> indexerSim.getAngularVelocityRPM() > 1.0)
        .debounce(0.2)
        .onTrue(Commands.runOnce(() -> indexerBeamBreak.setValue(true)));

    new Trigger(() -> indexerSim.getAngularVelocityRPM() > 3000 && indexerBeamBreak.getValue())
        .debounce(0.1)
        .onTrue(
            Commands.runOnce(
                () -> {
                  intakeBeamBreak.setValue(false);
                  indexerBeamBreak.setValue(false);
                }));
  }

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
