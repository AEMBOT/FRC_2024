package frc.robot.subsystems.notevision;

import org.littletonrobotics.junction.AutoLog;

public interface NoteVisionIO {
  @AutoLog
  public static class NoteVisionIOInputs {
    public boolean hasTarget = false;
    public double targetX = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(NoteVisionIOInputs inputs) {}
}
