package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;

public class Diagnostics {

  private Diagnostics() {}

  // wibble wobble switch between two commands for 1 second
  private static Command testWobble(Command minCommand, Command maxCommand) {
    // TODO check if repeatedly applied to both commands or just maxCommand
    return minCommand.andThen(maxCommand).repeatedly().withTimeout(1);
  }

  // run drivebase
  public static Command testDrivetrainCommand(Drive drive) {
    double minDrive = -drive.getMaxLinearSpeedMetersPerSec();
    double maxDrive = drive.getMaxLinearSpeedMetersPerSec();
    double avgDrive = (minDrive + maxDrive) / 2;
    double defaultDrive = 0;

    return drive
        .runVelocityCommand(() -> new ChassisSpeeds(minDrive, 0, 0))
        .andThen(drive.runVelocityCommand(() -> new ChassisSpeeds(maxDrive, 0, 0)))
        .andThen(drive.runVelocityCommand(() -> new ChassisSpeeds(avgDrive, 0, 0)))
        .andThen(
            testWobble(
                drive.runVelocityCommand(
                    () -> new ChassisSpeeds(avgDrive + ((maxDrive - minDrive) / 10), 0, 0)),
                drive.runVelocityCommand(
                    () -> new ChassisSpeeds(avgDrive - ((maxDrive - minDrive) / 10), 0, 0))))
        .andThen(drive.runVelocityCommand(() -> new ChassisSpeeds(defaultDrive, 0, 0)));
  }

  // run shooter
  public static Command testShooterCommand(Shooter shooter) {
    double minShooter = -Constants.ShooterConstants.shooterSpeedRPM;
    double maxShooter = Constants.ShooterConstants.shooterSpeedRPM;
    double avgShooter = (minShooter + maxShooter) / 2;
    double defaultShooter =
        Constants.ShooterConstants.shooterIdleRPM; // TODO check whether to go to this value or 0

    return shooter
        .setVelocityRPMCommand(minShooter)
        .andThen(shooter.setVelocityRPMCommand(maxShooter))
        .andThen(shooter.setVelocityRPMCommand(avgShooter))
        .andThen(
            testWobble(
                shooter.setVelocityRPMCommand(avgShooter + ((maxShooter - minShooter) / 10)),
                shooter.setVelocityRPMCommand(avgShooter - ((maxShooter - minShooter) / 10))))
        .andThen(shooter.setVelocityRPMCommand(defaultShooter));
  }

  // run pivot
  public static Command testPivotCommand(Pivot pivot) {
    double minPivot = Constants.PivotConstants.PIVOT_MIN_POS_RAD;
    double maxPivot = Constants.PivotConstants.PIVOT_MAX_POS_RAD;
    double avgPivot = (minPivot + maxPivot) / 2;
    double defaultPivot = minPivot; // TODO cofirm this value

    return pivot
        .setPositionCommand(() -> minPivot)
        .andThen(pivot.setPositionCommand(() -> maxPivot))
        .andThen(pivot.setPositionCommand(() -> avgPivot))
        .andThen(
            testWobble(
                pivot.setPositionCommand(() -> avgPivot + ((maxPivot - minPivot) / 10)),
                pivot.setPositionCommand(() -> avgPivot - ((maxPivot - minPivot) / 10))))
        .andThen(pivot.setPositionCommand(() -> defaultPivot));
  }

  // run indexer
  public static Command testIndexerCommand(Indexer indexer) {
    // TODO make this work like the others somehow
    return indexer.indexerOnIntakeOnCommand().andThen(indexer.indexerOffIntakeOffCommand());
  }

  // run climber

  public static Command testClimberCommand(Climber climber) {
    double minClimber = Constants.ClimberConstants.minExtendHardStop;
    double maxClimber = Constants.ClimberConstants.maxExtendSoftStop;
    double avgClimber = (minClimber + maxClimber) / 2;
    double defaultClimber = 0; // TODO cornfirm this value

    return climber
        .setPositionCommand(minClimber)
        .andThen(climber.setPositionCommand(maxClimber))
        .andThen(climber.setPositionCommand(avgClimber))
        .andThen(
            testWobble(
                climber.setPositionCommand(avgClimber + ((maxClimber - minClimber) / 10)),
                climber.setPositionCommand(avgClimber - ((maxClimber - minClimber) / 10))))
        .andThen(climber.setPositionCommand(defaultClimber));
  }

  // test for logs flashdrive

  // TODO get camera ip adresses,
  // is there clear video feed on all cameras

  // TODO can algorithm detect april tags
}
