package frc.robot.subsystems.indexer;

import frc.robot.subsystems.indexer.IndexerIO.IndexerIOInputs.MotorState;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public boolean intakeBeamBreakState = false;
    public boolean shooterBeamBreakState = true;

    public double shooterIndexerAppliedVolts = 0.0;
    public double intakeIndexerAppliedVolts = 0.0;

    public double[] intakeIndexerCurrentAmps = new double[] {};
    public double[] shooterIndexerCurrentAmps = new double[] {};

    public enum MotorState {
      IN,
      OFF,
      OUT
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IndexerIOInputs inputs) {}

  public default void setShooterIndexer(MotorState state) {}

  public default void setIntakeIndexer(MotorState state) {}
}
