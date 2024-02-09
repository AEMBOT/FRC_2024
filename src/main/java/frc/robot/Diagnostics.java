package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.pivot.Pivot;

public class Diagnostics {

  private Diagnostics() {}

  // run swerve motors

  public static Command testDrivetrainCommand(Drive drive) {
    return drive.runVelocityCommand(() -> new ChassisSpeeds(1, 1, 1));
  }

  // run shooter motors

  // run pivot motors

  public static Command testPivotMaxAngleCommand(Pivot pivot) {
    return pivot.goToAngle(Constants.PivotConstants.PIVOT_MAX_POS_RAD);
  }

  public static Command testPivotMinAngleCommand(Pivot pivot) {
    return pivot.goToAngle(Constants.PivotConstants.PIVOT_MIN_POS_RAD);
  }

  // run indexer motors

  // run climber motors

}
