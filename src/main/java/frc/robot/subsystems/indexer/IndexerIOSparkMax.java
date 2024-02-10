package frc.robot.subsystems.indexer;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.IndexerConstants.*;
import static frc.robot.Constants.IntakeConstants.*;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.indexer.IndexerIO.IndexerIOInputs.MotorState;

public class IndexerIOSparkMax implements IndexerIO {
  private final CANSparkMax indexerMotorTop = new CANSparkMax(indexerMotorPortTop, kBrushless);
  private final CANSparkMax indexerMotorBottom =
      new CANSparkMax(indexerMotorPortBottom, kBrushless);

  private final CANSparkMax intakeMotorTop = new CANSparkMax(intakeMotorPortTop, kBrushless);
  private final CANSparkMax intakeMotorBottom = new CANSparkMax(intakeMotorPortBottom, kBrushless);

  public DigitalInput intakeBeamBreak = new DigitalInput(intakeBeamBrake);
  public DigitalInput shooterBeamBreak = new DigitalInput(indexerBeamBrake);

  public IndexerIOSparkMax() {
    intakeMotorTop.restoreFactoryDefaults();
    intakeMotorBottom.restoreFactoryDefaults();
    indexerMotorTop.restoreFactoryDefaults();
    indexerMotorBottom.restoreFactoryDefaults();

    intakeMotorTop.setSmartCurrentLimit(20);
    intakeMotorBottom.setSmartCurrentLimit(20);
    indexerMotorTop.setSmartCurrentLimit(20);
    indexerMotorBottom.setSmartCurrentLimit(20);

    intakeMotorTop.setInverted(false);
    intakeMotorBottom.setInverted(false);
    indexerMotorTop.setInverted(false);
    indexerMotorBottom.setInverted(false);

    indexerMotorBottom.follow(indexerMotorTop);
    intakeMotorBottom.follow(intakeMotorTop);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.intakeBeamBreakState =
        !intakeBeamBreak.get(); // Beambreaks are normally on, need to invert
    inputs.shooterBeamBreakState = !shooterBeamBreak.get();
    inputs.intakeIndexerAppliedVolts =
        intakeMotorTop.getAppliedOutput() * intakeMotorTop.getBusVoltage();
    inputs.shooterIndexerAppliedVolts =
        indexerMotorTop.getAppliedOutput() * intakeMotorTop.getBusVoltage();
    inputs.intakeIndexerCurrentAmps =
        new double[] {intakeMotorTop.getOutputCurrent(), intakeMotorBottom.getOutputCurrent()};
    inputs.shooterIndexerCurrentAmps =
        new double[] {indexerMotorTop.getOutputCurrent(), indexerMotorBottom.getOutputCurrent()};
  }

  @Override
  public void setShooterIndexer(MotorState state) {
    switch (state) {
      case OFF:
        indexerMotorTop.setVoltage(0);
        break;
      case IN:
        indexerMotorTop.setVoltage(indexerMotorVoltage);
      case OUT:
        indexerMotorTop.setVoltage(-indexerMotorVoltage);
    }
  }

  @Override
  public void setIntakeIndexer(MotorState state) {
    switch (state) {
      case OFF:
        intakeMotorTop.setVoltage(0);
        break;
      case IN:
        intakeMotorTop.setVoltage(intakeMotorVoltage);
        break;
      case OUT:
        intakeMotorTop.setVoltage(-intakeMotorVoltage);
    }
  }
}
