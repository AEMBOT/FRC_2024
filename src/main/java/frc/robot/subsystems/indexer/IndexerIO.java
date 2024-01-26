package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public boolean intakeBeamBreak = false;
    public boolean shooterBeamBreak = true;

    public double shooterIndexerAppliedVolts = 0.0;
    public double intakeIndexerAppliedVolts = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IndexerIOInputs inputs) {}

  public default void runShooterIndexer(boolean state) {}

  public default void runIntakeIndexer(boolean state) {}
}
