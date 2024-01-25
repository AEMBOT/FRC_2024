package frc.robot.subsystems.Indexer;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants;

public class IndexerIOSparkMax implements IndexerIO {
    private final CANSparkMax indexerMotorTop =
            new CANSparkMax(Constants.IndexerConstants.indexerMotorPortTop, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax indexerMotorBottom =
            new CANSparkMax(Constants.IndexerConstants.indexerMotorPortBottom, CANSparkLowLevel.MotorType.kBrushless);

    public void SetVoltage(double voltage) {
        indexerMotorTop.setVoltage(voltage);
        indexerMotorBottom.setVoltage(voltage);
    }
}
