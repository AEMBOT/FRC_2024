package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public final class AprilTagConstants {
  public static enum CameraResolution {
    HIGH_RES,
    NORMAL
  }

  public static final Transform3d frontCamToRobot =
      new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0.0));
  public static final Transform3d leftCamToRobot =
      new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0.0));
  public static final Transform3d rightCamToRobot =
      new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0.0));
  public static final Matrix<N3, N1> highResSingleTagStdDev =
      VecBuilder.fill(0.25, 0.25, Double.MAX_VALUE);
  public static final Matrix<N3, N1> normalSingleTagStdDev =
      VecBuilder.fill(0.5, 0.5, Double.MAX_VALUE);
  public static final Matrix<N3, N1> highResMultiTagStdDev = VecBuilder.fill(0.1, 0.1, 2);
  public static final Matrix<N3, N1> normalMultiTagStdDev = VecBuilder.fill(0.2, 0.2, 4);
}
