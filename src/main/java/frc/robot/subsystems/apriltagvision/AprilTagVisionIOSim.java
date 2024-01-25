package frc.robot.subsystems.apriltagvision;

import static frc.robot.Constants.aprilTagFieldLayout;
import static frc.robot.subsystems.apriltagvision.AprilTagConstants.*;
import static org.photonvision.PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class AprilTagVisionIOSim implements AprilTagVisionIO {
  private final VisionSystemSim visionSim;

  // Forward Camera
  private final PhotonCameraSim frontCam;
  private final PhotonPoseEstimator frontPhotonPoseEstimator;

  // Left Side Camera
  private final PhotonCameraSim leftCam;
  private final PhotonPoseEstimator leftPhotonPoseEstimator;

  // Right Side Camera
  private final PhotonCameraSim rightCam;
  private final PhotonPoseEstimator rightPhotonPoseEstimator;

  public AprilTagVisionIOSim() {
    PhotonCamera front = new PhotonCamera("front");
    PhotonCamera left = new PhotonCamera("left");
    PhotonCamera right = new PhotonCamera("right");

    frontPhotonPoseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout, MULTI_TAG_PNP_ON_COPROCESSOR, front, frontCamToRobot);
    leftPhotonPoseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout, MULTI_TAG_PNP_ON_COPROCESSOR, left, leftCamToRobot);
    rightPhotonPoseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout, MULTI_TAG_PNP_ON_COPROCESSOR, right, rightCamToRobot);

    visionSim = new VisionSystemSim("main");
    visionSim.addAprilTags(aprilTagFieldLayout);

    SimCameraProperties frontCameraProp = new SimCameraProperties();
    frontCameraProp.setCalibration(1600, 1200, Rotation2d.fromDegrees(95.95));
    frontCameraProp.setCalibError(0.25, 0.10);
    frontCameraProp.setFPS(25);
    frontCameraProp.setAvgLatencyMs(50);
    frontCameraProp.setLatencyStdDevMs(15);

    SimCameraProperties sideCameraProp = new SimCameraProperties(); // Arducam OV9281, not Spinel
    sideCameraProp.setCalibration(1280, 800, Rotation2d.fromDegrees(84.47));
    sideCameraProp.setCalibError(0.25, 0.10);
    sideCameraProp.setFPS(40);
    sideCameraProp.setAvgLatencyMs(40);
    sideCameraProp.setLatencyStdDevMs(10);

    frontCam = new PhotonCameraSim(front, frontCameraProp);
    leftCam = new PhotonCameraSim(left, sideCameraProp);
    rightCam = new PhotonCameraSim(right, sideCameraProp);

    visionSim.addCamera(frontCam, frontCamToRobot);
    visionSim.addCamera(leftCam, leftCamToRobot);
    visionSim.addCamera(rightCam, rightCamToRobot);

    frontCam.enableDrawWireframe(true);
    leftCam.enableDrawWireframe(true);
    rightCam.enableDrawWireframe(true);
  }

  @Override
  public void updateInputs(AprilTagVisionIOInputs inputs) {
    inputs.visionPoses = getEstimatedPoseUpdates();
  }

  public void updatePose(Pose2d pose) {
    visionSim.update(pose);
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

  @AutoLogOutput
  public Field2d getSimDebugField() {
    return visionSim.getDebugField();
  }
}
