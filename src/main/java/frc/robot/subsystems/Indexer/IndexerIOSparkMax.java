package frc.robot.subsystems.Indexer;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
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

  @Override
  public void SetIndexerVoltage(double voltage) {
    indexerMotorTop.setVoltage(voltage);
    indexerMotorBottom.setVoltage(voltage);
  }

  @Override
  public void SetIntakeVoltage(double voltage) {
    intakeMotorTop.setVoltage(voltage);
    intakeMotorBottom.setVoltage(voltage);
  }
}
