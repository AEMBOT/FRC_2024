package frc.robot.subsystems.apriltagvision;

import static frc.robot.Constants.aprilTagFieldLayout;
import static frc.robot.subsystems.apriltagvision.AprilTagConstants.*;
import static org.photonvision.PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class AprilTagVisionIOReal implements AprilTagVisionIO {
  // Forward Camera
  private final PhotonCamera frontCam;
  private final PhotonPoseEstimator frontPhotonPoseEstimator;

  // Left Side Camera
  private final PhotonCamera leftCam;
  private final PhotonPoseEstimator leftPhotonPoseEstimator;

  // Right Side Camera
  private final PhotonCamera rightCam;
  private final PhotonPoseEstimator rightPhotonPoseEstimator;

  public AprilTagVisionIOReal() {
    frontCam = new PhotonCamera("front");
    frontPhotonPoseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout, MULTI_TAG_PNP_ON_COPROCESSOR, frontCam, frontCamToRobot);

    // Left Side Camera
    leftCam = new PhotonCamera("left");
    leftPhotonPoseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout, MULTI_TAG_PNP_ON_COPROCESSOR, leftCam, leftCamToRobot);

    // Right Side Camera
    rightCam = new PhotonCamera("right");
    rightPhotonPoseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout, MULTI_TAG_PNP_ON_COPROCESSOR, rightCam, rightCamToRobot);
  }

  @Override
  public void updateInputs(AprilTagVisionIOInputs inputs) {
    inputs.visionPoses = getEstimatedPoseUpdates();
  }

  public ArrayList<Pair<EstimatedRobotPose, Matrix<N3, N1>>> getEstimatedPoseUpdates() {
    ArrayList<Pair<EstimatedRobotPose, Matrix<N3, N1>>> results = new ArrayList<>(3);

    Optional<EstimatedRobotPose> pose = frontPhotonPoseEstimator.update();
    pose.ifPresent(
        estPose ->
            results.add(
                Pair.of(estPose, getEstimationStdDevs(estPose, CameraResolution.HIGH_RES))));
    pose = leftPhotonPoseEstimator.update();
    pose.ifPresent(
        estPose ->
            results.add(Pair.of(estPose, getEstimationStdDevs(estPose, CameraResolution.NORMAL))));
    pose = rightPhotonPoseEstimator.update();
    pose.ifPresent(
        estPose ->
            results.add(Pair.of(estPose, getEstimationStdDevs(estPose, CameraResolution.NORMAL))));

    results.sort(Comparator.comparingDouble(p -> p.getFirst().timestampSeconds));

    return results;
  }
}
