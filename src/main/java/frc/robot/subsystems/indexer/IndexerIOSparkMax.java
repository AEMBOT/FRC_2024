package frc.robot.subsystems.indexer;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class IndexerIOSparkMax implements IndexerIO {
  private final CANSparkMax indexerMotorTop =
      new CANSparkMax(
          Constants.IndexerConstants.indexerMotorPortTop, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax indexerMotorBottom =
      new CANSparkMax(
          Constants.IndexerConstants.indexerMotorPortBottom, CANSparkLowLevel.MotorType.kBrushless);

  private final CANSparkMax intakeMotorTop =
      new CANSparkMax(
          Constants.IntakeConstants.intakeMotorPortTop, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax intakeMotorBottom =
      new CANSparkMax(
          Constants.IntakeConstants.intakeMotorPortBottom, CANSparkLowLevel.MotorType.kBrushless);

  public DigitalInput intakeBeamBreak = new DigitalInput(Constants.IntakeConstants.intakeBeamBrake);
  public DigitalInput shooterBeamBreak =
      new DigitalInput(Constants.IndexerConstants.indexerBeamBrake);

  public DigitalInput intakeBeamBreak;
  public DigitalInput shooterBeamBreak;

  public boolean intakeBeamBreakState = false;
  public boolean shooterBeamBreakState = true;

  public IndexerIOInputs.MotorState indexerMotorsState = IndexerIOInputs.MotorState.OFF;
  public IndexerIOInputs.MotorState intakeMotorsState = IndexerIOInputs.MotorState.OFF;

  @Override
  public void setShooterIndexer(IndexerIOInputs.MotorState state) {
    indexerMotorsState = state;
  }

  @Override
  public void setIntakeIndexer(IndexerIOInputs.MotorState state) {
    intakeMotorsState = state;
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.intakeBeamBreakState = !intakeBeamBreak.get();
    inputs.shooterBeamBreakState = !shooterBeamBreak.get();

    switch (intakeMotorsState) {
      case OFF:
        intakeMotorBottom.setVoltage(0);
        intakeMotorTop.setVoltage(0);
        break;
      case IN:
        intakeMotorBottom.setVoltage(Constants.IntakeConstants.intakeMotorVoltage);
        intakeMotorTop.setVoltage(Constants.IntakeConstants.intakeMotorVoltage);
        break;
      case OUT:
        intakeMotorBottom.setVoltage(-Constants.IntakeConstants.intakeMotorVoltage);
        intakeMotorTop.setVoltage(-Constants.IntakeConstants.intakeMotorVoltage);
    }
  }
}
