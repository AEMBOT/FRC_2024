package frc.robot.subsystems.notevision;

import org.littletonrobotics.junction.AutoLog;

public interface NoteVisionIO {
  @AutoLog
  public static class NoteVisionIOInputs {
    public boolean hasTarget = false;
    public boolean lastHasTarget = true;
    public double targetX = 0.0;
    public double lastTargetX = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(NoteVisionIOInputs inputs) {}
}
