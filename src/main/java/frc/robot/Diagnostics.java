package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.repeatingSequence;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;

public class Diagnostics {

  private Diagnostics() {}

  // wibble wobble switch between two commands for 1 second
  private static Command testWobble(Command minCommand, Command maxCommand) {
    return repeatingSequence(
            minCommand.withTimeout(0.25), WaitCommand(0.25), maxCommand.withTimeout(0.25))
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
              .withTimeout(1)
              .andThen(drive.singleMotorDriveTest(modules[i], -4))
              .withTimeout(1)
              .andThen(drive.singleMotorDriveTest(modules[i], 0))
              .withTimeout(1)
              .andThen(
                  testWobble(
                      drive.singleMotorDriveTest(modules[i], 2),
                      drive.singleMotorDriveTest(modules[i], -2)))
              .andThen(drive.singleMotorDriveTest(modules[i], 0))
              .withTimeout(1);
    }
    for (int i = 0; i < 4; ++i) {
      driveCommand =
          driveCommand
              .andThen(drive.singleMotorSteerTest(modules[i], 4))
              .withTimeout(1)
              .andThen(drive.singleMotorSteerTest(modules[i], -4))
              .withTimeout(1)
              .andThen(drive.singleMotorSteerTest(modules[i], 0))
              .withTimeout(1)
              .andThen(
                  testWobble(
                      drive.singleMotorSteerTest(modules[i], 2),
                      drive.singleMotorSteerTest(modules[i], -2)))
              .andThen(drive.singleMotorSteerTest(modules[i], 0))
              .withTimeout(1);
    }
    return driveCommand;
  }

  // run shooter
  public static Command testShooterCommand(Shooter shooter) {
    double minShooter = -Constants.ShooterConstants.shooterSpeedRPM;
    double maxShooter = Constants.ShooterConstants.shooterSpeedRPM;
    double defaultShooter =
        Constants.ShooterConstants.shooterIdleRPM; // TODO check whether to go to this value or 0

    return shooter
        .setVelocityRPMCommand(minShooter)
        .withTimeout(1)
        .andThen(shooter.setVelocityRPMCommand(maxShooter))
        .withTimeout(1)
        .andThen(shooter.stopCommand())
        .andThen(testWobble(shooter.setVoltageCommand(4), shooter.setVoltageCommand(-4)))
        .andThen(shooter.setVelocityRPMCommand(defaultShooter))
        .withTimeout(1);
  }

  // run pivot
  public static Command testPivotCommand(Pivot pivot) {
    double minPivot = Constants.PivotConstants.PIVOT_MIN_POS_RAD;
    double maxPivot = Constants.PivotConstants.PIVOT_MAX_POS_RAD;
    double avgPivot = (minPivot + maxPivot) / 2;
    double defaultPivot = minPivot; // TODO cofirm this value

    return pivot
        .setPositionCommand(() -> minPivot)
        .withTimeout(1)
        .andThen(pivot.setPositionCommand(() -> maxPivot))
        .withTimeout(1)
        .andThen(pivot.setPositionCommand(() -> avgPivot))
        .withTimeout(1)
        .andThen(
            testWobble(
                pivot.setPositionCommand(() -> avgPivot + ((maxPivot - minPivot) / 10)),
                pivot.setPositionCommand(() -> avgPivot - ((maxPivot - minPivot) / 10))))
        .andThen(pivot.setPositionCommand(() -> defaultPivot))
        .withTimeout(1);
  }

  // run indexer
  public static Command testIndexerCommand(Indexer indexer) {
    return indexer
        .intakeInCommand()
        .alongWith(indexer.indexerInCommand())
        .andThen(indexer.intakeOutCommand().alongWith(indexer.indexerOutCommand()))
        .andThen(indexer.indexerOffIntakeOffCommand())
        .andThen(
            testWobble(
                indexer.intakeInCommand().alongWith(indexer.indexerInCommand()),
                indexer.intakeOutCommand().alongWith(indexer.indexerOutCommand())))
        .andThen(indexer.indexerOffIntakeOffCommand());
  }

  // run climber
  public static Command testClimberCommand(Climber climber) {
    double minClimber = Constants.ClimberConstants.minExtendHardStop;
    double maxClimber = Constants.ClimberConstants.maxExtendSoftStop;
    double avgClimber = (minClimber + maxClimber) / 2;
    double defaultClimber = 0; // TODO cornfirm this value

    return climber
        .setPositionCommand(minClimber)
        .withTimeout(1)
        .andThen(climber.setPositionCommand(maxClimber))
        .withTimeout(1)
        .andThen(climber.setPositionCommand(avgClimber))
        .withTimeout(1)
        .andThen(
            testWobble(
                climber.setPositionCommand(avgClimber + ((maxClimber - minClimber) / 10)),
                climber.setPositionCommand(avgClimber - ((maxClimber - minClimber) / 10))))
        .andThen(climber.setPositionCommand(defaultClimber))
        .withTimeout(1);
  }

  // TODO get camera ip adresses
  // is there clear video feed on all cameras

  // TODO can algorithm detect april tags
}
