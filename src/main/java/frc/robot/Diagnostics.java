package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.repeatingSequence;
import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;

public class Diagnostics {

  private Diagnostics() {}

  private static final double wobbleStepTimeout = Constants.DiagnosticConstants.wobbleStepTimeout;
  private static final double rangeOfMotionTimeout =
      Constants.DiagnosticConstants.rangeOfMotionTimeout;
  private static final double rangeOfMotionDelay = Constants.DiagnosticConstants.rangeOfMotionDelay;
  private static final double driveTestVolts = Constants.DiagnosticConstants.driveTestVolts;
  private static final double driveTestWobbleVolts =
      Constants.DiagnosticConstants.driveTestWobbleVolts;
  private static final double driveATWTestVolts = Constants.DiagnosticConstants.driveATWTestVolts;
  private static final double driveATWTestWobbleVolts =
      Constants.DiagnosticConstants.driveATWTestWobbleVolts;

  // Alternate / "wobble" between two commands
  private static Command testWobble(Command minCommand, Command maxCommand) {
    return repeatingSequence(
            minCommand.withTimeout(wobbleStepTimeout), maxCommand.withTimeout(wobbleStepTimeout))
        .withTimeout(2);
  }

  private static Command WaitCommand(double d) {
    return Commands.waitSeconds(d);
  }

  // run drivebase
  public static Command testDrivetrainCommand(Drive drive) {
    Command driveCommand = WaitCommand(0);
    frc.robot.subsystems.drive.Module[] modules = drive.returnMotors();
    for (int i = 0; i < 4; ++i) {
      driveCommand =
          driveCommand
              .andThen(drive.singleMotorDriveTest(modules[i], driveTestVolts))
              .withTimeout(rangeOfMotionTimeout)
              .andThen(drive.singleMotorDriveTest(modules[i], -driveTestVolts))
              .withTimeout(rangeOfMotionTimeout)
              .andThen(drive.singleMotorDriveTest(modules[i], 0))
              .withTimeout(rangeOfMotionTimeout)
              .andThen(
                  testWobble(
                      drive.singleMotorDriveTest(modules[i], driveTestWobbleVolts),
                      drive.singleMotorDriveTest(modules[i], -driveTestWobbleVolts)))
              .andThen(drive.singleMotorDriveTest(modules[i], 0));
    }
    for (int i = 0; i < 4; ++i) {
      driveCommand =
          driveCommand
              .andThen(drive.singleMotorSteerTest(modules[i], driveTestVolts))
              .withTimeout(rangeOfMotionTimeout)
              .andThen(drive.singleMotorSteerTest(modules[i], -driveTestVolts))
              .withTimeout(rangeOfMotionTimeout)
              .andThen(drive.singleMotorSteerTest(modules[i], 0))
              .withTimeout(rangeOfMotionTimeout)
              .andThen(
                  testWobble(
                      drive.singleMotorSteerTest(modules[i], driveTestWobbleVolts),
                      drive.singleMotorSteerTest(modules[i], -driveTestWobbleVolts)))
              .andThen(drive.singleMotorSteerTest(modules[i], 0));
    }
    for (int i = 0; i < 4; ++i) {
      driveCommand =
          driveCommand
              .andThen(drive.singleMotorATWTest(modules[i], driveATWTestVolts))
              .withTimeout(rangeOfMotionTimeout)
              .andThen(drive.singleMotorATWTest(modules[i], -driveATWTestVolts))
              .withTimeout(rangeOfMotionTimeout)
              .andThen(drive.singleMotorATWTest(modules[i], 0))
              .withTimeout(rangeOfMotionTimeout)
              .andThen(
                  testWobble(
                      drive.singleMotorATWTest(modules[i], driveATWTestWobbleVolts),
                      drive.singleMotorATWTest(modules[i], -driveATWTestWobbleVolts)))
              .andThen(drive.singleMotorATWTest(modules[i], 0));
    }
    return driveCommand;
  }

  // run shooter
  public static Command testShooterCommand(Shooter shooter) {
    return Commands.sequence(
        waitSeconds(rangeOfMotionDelay), // as an example
        shooter.setVelocityRPMCommand(0).until(() -> shooter.isAtShootSpeed()),
        waitSeconds(rangeOfMotionDelay),
        shooter
            .setVelocityRPMCommand(Constants.ShooterConstants.shooterSpeedRPM)
            .until(() -> shooter.isAtShootSpeed()),
        waitSeconds(rangeOfMotionDelay),
        // Don't do wobble on shooter, unnecessary
        shooter
            .setVelocityRPMCommand(Constants.ShooterConstants.shooterIdleRPM)
            .until(() -> shooter.isAtShootSpeed()),
        waitSeconds(rangeOfMotionDelay));
  }

  // run pivot
  public static Command testPivotCommand(Pivot pivot) {
    double minPivot = Constants.PivotConstants.PIVOT_MIN_POS_RAD;
    double maxPivot = Constants.PivotConstants.PIVOT_MAX_POS_RAD;
    double avgPivot = (minPivot + maxPivot) / 2;

    // TODO rework this because i dont think its how its supposed to be

    return Commands.sequence(
        waitSeconds(rangeOfMotionDelay), // as an example
        pivot.setPositionCommand(() -> minPivot).until(() -> pivot.atGoal()),
        waitSeconds(rangeOfMotionDelay),
        pivot.setPositionCommand(() -> maxPivot).until(() -> pivot.atGoal()),
        waitSeconds(rangeOfMotionDelay),
        pivot.setPositionCommand(() -> avgPivot).until(() -> pivot.atGoal()),
        waitSeconds(rangeOfMotionDelay),
        testWobble(
            pivot.setPositionCommand(() -> avgPivot + ((maxPivot - minPivot) / 10)),
            pivot.setPositionCommand(() -> avgPivot - ((maxPivot - minPivot) / 10))),
        waitSeconds(rangeOfMotionDelay),
        pivot.setPositionCommand(() -> avgPivot).until(() -> pivot.atGoal()));
  }

  // run indexer
  public static Command testIndexerCommand(Indexer indexer) {
    return indexer
        .intakeInCommand()
        .withTimeout(rangeOfMotionTimeout)
        .andThen(indexer.intakeOutCommand())
        .withTimeout(rangeOfMotionTimeout)
        .andThen(
            indexer
                .indexerInCommand()
                .withTimeout(rangeOfMotionTimeout)
                .andThen(indexer.indexerOutCommand()))
        .withTimeout(rangeOfMotionTimeout)
        .andThen(indexer.intakeStopCommand())
        .andThen(indexer.indexerStopCommand())
        .withTimeout(rangeOfMotionTimeout)
        .andThen(
            testWobble(
                indexer.intakeInCommand().andThen(indexer.indexerInCommand()),
                indexer.intakeOutCommand().andThen(indexer.indexerOutCommand())))
        .andThen(indexer.intakeStopCommand())
        .andThen(indexer.indexerStopCommand());
  }

  // run climber
  public static Command testClimberCommand(Climber climber) {
    double minClimberHeight = Constants.ClimberConstants.minExtendHardStop;
    double maxClimberHeight = Constants.ClimberConstants.maxExtendSoftStop;
    double avgClimberHeight = (minClimberHeight + maxClimberHeight) / 2;

    return climber
        .getHomingCommand()
        .andThen(climber.setLeftPositionCommand(minClimberHeight))
        .withTimeout(rangeOfMotionTimeout)
        .andThen(climber.setLeftPositionCommand(maxClimberHeight))
        .withTimeout(rangeOfMotionTimeout)
        .andThen(climber.setLeftPositionCommand(avgClimberHeight))
        .withTimeout(rangeOfMotionTimeout)
        .andThen(
            testWobble(
                climber.setLeftPositionCommand(
                    avgClimberHeight + ((maxClimberHeight - minClimberHeight) / 10)),
                climber.setLeftPositionCommand(
                    avgClimberHeight - ((maxClimberHeight - minClimberHeight) / 10))))
        .andThen(climber.getHomingCommand())
        .withTimeout(rangeOfMotionTimeout)
        .andThen(climber.setRightPositionCommand(minClimberHeight))
        .withTimeout(rangeOfMotionTimeout)
        .andThen(climber.setRightPositionCommand(maxClimberHeight))
        .withTimeout(rangeOfMotionTimeout)
        .andThen(climber.setRightPositionCommand(avgClimberHeight))
        .withTimeout(rangeOfMotionTimeout)
        .andThen(
            testWobble(
                climber.setRightPositionCommand(
                    avgClimberHeight + ((maxClimberHeight - minClimberHeight) / 10)),
                climber.setRightPositionCommand(
                    avgClimberHeight - ((maxClimberHeight - minClimberHeight) / 10))))
        .andThen(climber.getHomingCommand())
        .withTimeout(rangeOfMotionTimeout);
  }

  // TODO get camera ip adresses
  // check if there is clear video feed on all cameras

  public static Command testAprilTagVisionCommand(Drive drive, SerialPort prettyLights) {

    // TODO get positions

    // pose2d of the robot when april tag is shown in the left camera
    Pose2d leftCamPosition = new Pose2d(0, 0, new Rotation2d(0));
    // pose2d of the robot when april tag is shown in the center camera
    Pose2d centerCamPosition = new Pose2d(0, 0, new Rotation2d(0));
    // pose2d of the robot when april tag is shown in the right camera
    Pose2d rightCamPosition = new Pose2d(0, 0, new Rotation2d(0));

    return checkPose(drive, leftCamPosition, prettyLights)
        .andThen(checkPose(drive, centerCamPosition, prettyLights))
        .andThen(checkPose(drive, rightCamPosition, prettyLights));
  }

  private static Command checkPose(Drive drive, Pose2d camPosition, SerialPort prettyLights) {

    // TODO get good lenience values

    double positionLenience = 1;
    double rotationLenience = 40;

    return run(() -> prettyLights.writeString("r"))
        .until(
            () ->
                drive.getPose().getX() < camPosition.getX() + positionLenience
                    && drive.getPose().getX() > camPosition.getX() - positionLenience
                    && drive.getPose().getY() < camPosition.getY() + positionLenience
                    && drive.getPose().getY() > camPosition.getY() - positionLenience
                    && drive.getPose().getRotation().getDegrees()
                        < camPosition.getRotation().getDegrees() + rotationLenience
                    && drive.getPose().getRotation().getDegrees()
                        > camPosition.getRotation().getDegrees() - rotationLenience)
        .andThen(run(() -> prettyLights.writeString("g")))
        .withTimeout(1);
  }
}
