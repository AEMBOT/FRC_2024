package frc.robot.subsystems.Indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

  @AutoLog
  public static class IndexerIOinputs {
    public double appliedVolts = 0.0;
    public boolean gamepieceIn = true;
  }

  public default void SetIndexerVoltage(double voltage) {}

  public default void SetIntakeVoltage(double voltage) {}
}
