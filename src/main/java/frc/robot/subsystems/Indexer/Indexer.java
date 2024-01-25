package frc.robot.subsystems.Indexer;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
  private CANSparkMax indexerMotorTop =
      new CANSparkMax(Constants.IndexerConstants.indexerMotorPortTop, MotorType.kBrushless);
  private CANSparkMax indexerMotorBottom =
      new CANSparkMax(Constants.IndexerConstants.indexerMotorPortBottom, MotorType.kBrushless);

  public void IndexerIn() {
    indexerMotorTop.setVoltage(Constants.IndexerConstants.indexerMotorVoltage);
    indexerMotorBottom.setVoltage(Constants.IndexerConstants.indexerMotorVoltage);
  }

  public void IndexerOut() {
    indexerMotorTop.setVoltage(-Constants.IndexerConstants.indexerMotorVoltage);
    indexerMotorBottom.setVoltage(-Constants.IndexerConstants.indexerMotorVoltage);
  }

  public void IndexerStop() {
    indexerMotorTop.setVoltage(0);
    indexerMotorBottom.setVoltage(0);
  }
}
