package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;

public class Diagnostics {

  private Diagnostics() {}

  private static final double wobbleStepTimeout = 0.25;
  private static final double rangeOfMotionTimeout = 1;
  private static final double rangeOfMotionDelay = 0.2;

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
              .andThen(drive.singleMotorDriveTest(modules[i], 4))
              .withTimeout(rangeOfMotionTimeout)
              .andThen(drive.singleMotorDriveTest(modules[i], -4))
              .withTimeout(rangeOfMotionTimeout)
              .andThen(drive.singleMotorDriveTest(modules[i], 0))
              .withTimeout(rangeOfMotionTimeout)
              .andThen(
                  testWobble(
                      drive.singleMotorDriveTest(modules[i], 2),
                      drive.singleMotorDriveTest(modules[i], -2)))
              .andThen(drive.singleMotorDriveTest(modules[i], 0));
    }
    for (int i = 0; i < 4; ++i) {
      driveCommand =
          driveCommand
              .andThen(drive.singleMotorSteerTest(modules[i], 4))
              .withTimeout(rangeOfMotionTimeout)
              .andThen(drive.singleMotorSteerTest(modules[i], -4))
              .withTimeout(rangeOfMotionTimeout)
              .andThen(drive.singleMotorSteerTest(modules[i], 0))
              .withTimeout(rangeOfMotionTimeout)
              .andThen(
                  testWobble(
                      drive.singleMotorSteerTest(modules[i], 2),
                      drive.singleMotorSteerTest(modules[i], -2)))
              .andThen(drive.singleMotorSteerTest(modules[i], 0));
    }
    return driveCommand;
  }

  // run shooter
  public static Command testShooterCommand(Shooter shooter) {
    return Commands.sequence(
        waitSeconds(rangeOfMotionDelay), // as an example
        shooter.setVelocityRPMCommand(0).until(() -> shooter.isAtShootSpeed()),
        waitSeconds(rangeOfMotionDelay),
        shooter.setVelocityRPMCommand(Constants.ShooterConstants.shooterSpeedRPM).until(() -> shooter.isAtShootSpeed()),
        waitSeconds(rangeOfMotionDelay),
        // Don't do wobble on shooter, unnecessary
        shooter.setVelocityRPMCommand(Constants.ShooterConstants.shooterIdleRPM).until(() -> shooter.isAtShootSpeed()),
        waitSeconds(rangeOfMotionDelay));
  }

  // run pivot
  public static Command testPivotCommand(Pivot pivot) {
    double minPivot = Constants.PivotConstants.PIVOT_MIN_POS_RAD;
    double maxPivot = Constants.PivotConstants.PIVOT_MAX_POS_RAD;
    double avgPivot = (minPivot + maxPivot) / 2;

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
        .andThen(indexer.indexerInCommand())
        .withTimeout(rangeOfMotionTimeout)
        .andThen(indexer.intakeOutCommand().andThen(indexer.indexerOutCommand()))
        .withTimeout(rangeOfMotionTimeout)
        .andThen(indexer.indexerOffIntakeOffCommand())
        .andThen(
            testWobble(
                indexer.intakeInCommand().andThen(indexer.indexerInCommand()),
                indexer.intakeOutCommand().andThen(indexer.indexerOutCommand())))
        .andThen(indexer.indexerOffIntakeOffCommand());
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
  // is there clear video feed on all cameras

  // TODO can algorithm detect april tags
}
