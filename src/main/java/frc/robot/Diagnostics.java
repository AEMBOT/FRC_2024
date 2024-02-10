package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;

public class Diagnostics {

  private Diagnostics() {}

  // run drivebase
  public static Command testDrivetrainCommand(Drive drive) {
    return drive.runVelocityCommand(() -> new ChassisSpeeds(1, 1, 1));
  }

  // run shooter
  public static Command testShooterCommand(Shooter shooter) {
    return shooter.setVelocityRPMCommand(60);
  }

  // run pivot
  public static Command testPivotCommand(Pivot pivot) {
    return pivot
        .goToAngle(Constants.PivotConstants.PIVOT_MAX_POS_RAD)
        .andThen(pivot.goToAngle(Constants.PivotConstants.PIVOT_MIN_POS_RAD));
  }

  // run indexer
  public static Command testIndexerCommand(Indexer indexer) {
    return indexer.indexerOnIntakeOnCommand()
    .andThen(indexer.indexerOffIntakeOffCommand());
  }

  // run climber
  //    schlawg there aint no climber
}
