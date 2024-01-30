package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public boolean intakeBeamBreakState = false;
    public boolean shooterBeamBreakState = true;

    public double shooterIndexerAppliedVolts = 0.0;
    public double intakeIndexerAppliedVolts = 0.0;

    public enum MotorState {
      IN,
      OFF,
      OUT
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IndexerIOInputs inputs) {}

  public default void setShooterIndexer(IndexerIOInputs.MotorState state) {}

  public default void setIntakeIndexer(IndexerIOInputs.MotorState state) {}
}
