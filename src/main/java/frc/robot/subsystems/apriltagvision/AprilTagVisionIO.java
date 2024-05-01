package frc.robot.subsystems.apriltagvision;

import static frc.robot.Constants.aprilTagFieldLayout;
import static frc.robot.subsystems.apriltagvision.AprilTagConstants.*;
import static frc.robot.subsystems.apriltagvision.AprilTagConstants.normalMultiTagStdDev;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;

public interface AprilTagVisionIO {
  @AutoLog
  public static class AprilTagVisionIOInputs {
    public Pose3d[] visionPoses =
        List.of(new Pose3d(), new Pose3d(), new Pose3d()).toArray(new Pose3d[0]);
    public double[] timestamps = new double[3];
    public double[] latency = new double[3];
    public double[] visionStdDevs = new double[9];
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(AprilTagVisionIOInputs inputs) {}

  /** Update the reference pose of the vision system. Currently only used in sim. */
  public default void updatePose(Pose2d pose) {}

  /**
   * The standard deviations of the estimated poses from vision cameras, for use with {@link
   * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   */
  default Matrix<N3, N1> getEstimationStdDevs(
      EstimatedRobotPose estimatedPose, CameraResolution resolution) {
    var estStdDevs =
        switch (resolution) {
          case HIGH_RES -> highResSingleTagStdDev;
          case NORMAL -> normalSingleTagStdDev;
        };
    var targets = estimatedPose.targetsUsed;
    int numTags = 0;
    double avgDist = 0;
    for (var tgt : targets) {
      var tagPose = aprilTagFieldLayout.getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty()) continue;
      numTags++;
      avgDist +=
          tagPose
              .get()
              .toPose2d()
              .minus(estimatedPose.estimatedPose.toPose2d())
              .getTranslation()
              .getNorm();
    }

    if (numTags == 0) return estStdDevs;
    avgDist /= numTags;

    // Decrease std devs if multiple targets are visible
    if (numTags > 1
        && avgDist
            > switch (resolution) {
              case HIGH_RES -> 8;
              case NORMAL -> 5;
            }) {
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    } else {
      estStdDevs =
          switch (resolution) {
            case HIGH_RES -> highResMultiTagStdDev;
            case NORMAL -> normalMultiTagStdDev;
          };
    }
    // Increase std devs based on (average) distance
    if (numTags == 1
        && avgDist
            > switch (resolution) {
              case HIGH_RES -> 6;
              case NORMAL -> 4;
            }) {
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    } else {
      estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 20));
    }

    return estStdDevs;
  }
}
