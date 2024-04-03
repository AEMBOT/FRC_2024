package frc.robot.subsystems.notevision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class NoteVisionIOReal implements NoteVisionIO {
  private final PhotonCamera intakeCam;

  public NoteVisionIOReal() {
    intakeCam = new PhotonCamera("intake");
  }

  @Override
  public void updateInputs(NoteVisionIOInputs inputs) {
    var result = intakeCam.getLatestResult();
    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();
      inputs.hasTarget = true;
      inputs.lastTargetX = inputs.targetX;
      inputs.targetX = target.getYaw();
    } else {
      inputs.hasTarget = false;
      inputs.lastTargetX = inputs.targetX;
      inputs.targetX = 0.0;
    }
  }
}
