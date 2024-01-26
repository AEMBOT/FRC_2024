package frc.robot.subsystems.indexer;

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
  public void runShooterIndexer(boolean state) {
    if (state) {
      indexerMotorTop.setVoltage(Constants.IndexerConstants.indexerMotorVoltage);
      indexerMotorBottom.setVoltage(Constants.IndexerConstants.indexerMotorVoltage);
    } else {
      indexerMotorTop.setVoltage(0);
      indexerMotorBottom.setVoltage(0);
    }
  }

  public void reverseShooterIndexer() {
    indexerMotorTop.setVoltage(-Constants.IndexerConstants.indexerMotorVoltage);
    indexerMotorBottom.setVoltage(-Constants.IndexerConstants.indexerMotorVoltage);
  }

  @Override
  public void runIntakeIndexer(boolean state) {
    if (state) {
      intakeMotorTop.setVoltage(Constants.IntakeConstants.intakeMotorVoltage);
      intakeMotorBottom.setVoltage(Constants.IntakeConstants.intakeMotorVoltage);
    } else {
      intakeMotorBottom.setVoltage(0);
      intakeMotorTop.setVoltage(0);
    }
  }

  public void reverseIntakeIndexer() {
    intakeMotorTop.setVoltage(-Constants.IntakeConstants.intakeMotorVoltage);
    intakeMotorBottom.setVoltage(-Constants.IntakeConstants.intakeMotorVoltage);
  }
}
