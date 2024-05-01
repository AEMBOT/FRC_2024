package frc.robot.commands;

import static frc.robot.Constants.FieldConstants.getPass;
import static frc.robot.Constants.FieldConstants.getSpeaker;
import static frc.robot.Constants.shootingSpeakerConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class SpeakerCommands {
  private static final double DEADBAND = 0.05;

  public static final InterpolatingDoubleTreeMap interpolator = new InterpolatingDoubleTreeMap();

  static {
    for (double[] shooterInterpolationPoint : shooterInterpolationPoints) {
      interpolator.put(shooterInterpolationPoint[0], shooterInterpolationPoint[1]);
    }
  }

  public static Command shootSpeaker(
      Drive drive, Pivot pivot, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    DoubleSupplier distance = () -> getSpeaker().getDistance(drive.getPose().getTranslation());
    PIDController pidController = new PIDController(kP, kI, kD);
    pidController.enableContinuousInput(-Math.PI, Math.PI);
    Command driveTrainCommand =
        Commands.run(
            () -> {
              Rotation2d targetAngle =
                  getSpeaker().minus(drive.getPose().getTranslation()).getAngle();
              Logger.recordOutput("Target Angle", targetAngle);
              // Apply deadband
              double linearMagnitude =
                  MathUtil.applyDeadband(
                      Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
              Rotation2d linearDirection =
                  new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
              double omega =
                  pidController.calculate(
                      drive.getRotation().getRadians(), targetAngle.getRadians());

              // Square values
              linearMagnitude = linearMagnitude * linearMagnitude;

              // Calcaulate new linear velocity
              Translation2d linearVelocity =
                  new Pose2d(new Translation2d(), linearDirection)
                      .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                      .getTranslation();

              // Convert to field relative speeds & send command
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      isFlipped
                          ? -linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec()
                          : linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      isFlipped
                          ? -linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec()
                          : linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega,
                      drive.getRotation()));

              Logger.recordOutput(
                  "Interpolator Distance", interpolator.get(distance.getAsDouble()));
            },
            drive);

    return driveTrainCommand.alongWith(
        new ProxyCommand(pivot.setPositionCommand(() -> interpolator.get(distance.getAsDouble()))));
  }

  public static Command shootPass(
      Drive drive,
      Pivot pivot,
      Shooter shooter,
      Indexer indexer,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {
    DoubleSupplier distance = () -> getPass().getDistance(drive.getPose().getTranslation());
    PIDController pidController = new PIDController(kP, kI, kD);
    pidController.enableContinuousInput(-Math.PI, Math.PI);
    Command driveTrainCommand =
        Commands.run(
            () -> {
              Rotation2d targetAngle = getPass().minus(drive.getPose().getTranslation()).getAngle();
              Logger.recordOutput("Target Angle", targetAngle);
              // Apply deadband
              double linearMagnitude =
                  MathUtil.applyDeadband(
                      Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
              Rotation2d linearDirection =
                  new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
              double omega =
                  pidController.calculate(
                      drive.getRotation().getRadians(), targetAngle.getRadians());

              // Square values
              linearMagnitude = linearMagnitude * linearMagnitude;

              // Calcaulate new linear velocity
              Translation2d linearVelocity =
                  new Pose2d(new Translation2d(), linearDirection)
                      .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                      .getTranslation();

              // Convert to field relative speeds & send command
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      isFlipped
                          ? -linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec()
                          : linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      isFlipped
                          ? -linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec()
                          : linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega,
                      drive.getRotation()));
            },
            drive);

    return Commands.parallel(
        driveTrainCommand
            .until(
                () ->
                    drive.getRobotRelativeSpeeds().omegaRadiansPerSecond < 0.05
                        && shooter.isAtShootSpeed()
                        && pivot.atGoal())
            .andThen(indexer.shootCommand().alongWith(drive.stopWithXCommand())),
        new ProxyCommand(pivot.setPositionCommand(() -> Units.degreesToRadians(60))),
        new ProxyCommand(shooter.setVelocityRPMCommand(4500)));
  }

  public static Command intakeNote(
      Drive drive,
      Indexer indexer,
      Pivot pivot,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    PIDController pidController = new PIDController(0.1, 0, 0);
    Command driveTrainCommand =
        Commands.run(
            () -> {
              // Apply deadband
              double linearMagnitude =
                  MathUtil.applyDeadband(
                      Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
              Rotation2d linearDirection =
                  new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              double omega;
              if (drive.hasNoteTarget()) {
                omega = pidController.calculate(drive.getNoteLocation(), 0);
                Logger.recordOutput("NoteVision/PIDCalculated", omega);
              } else {
                omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);
                Logger.recordOutput("NoteVision/PIDCalculated", 0);
              }

              // Square values
              linearMagnitude = linearMagnitude * linearMagnitude;

              // Calcaulate new linear velocity
              Translation2d linearVelocity =
                  new Pose2d(new Translation2d(), linearDirection)
                      .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                      .getTranslation();

              // Convert to field relative speeds & send command
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      isFlipped
                          ? -linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec()
                          : linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      isFlipped
                          ? -linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec()
                          : linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega,
                      drive.getRotation()));
            },
            drive);

    return driveTrainCommand.alongWith(indexer.getDefault(pivot::inHandoffZone));
  }

  public static Command intakeNoteVectorLock(
      Drive drive,
      Indexer indexer,
      Pivot pivot,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    PIDController pidController = new PIDController(0.1, 0, 0);
    Command driveTrainCommand =
        Commands.sequence(
            Commands.race(
                Commands.run(
                    () -> {
                      // Apply deadband
                      double linearMagnitude =
                          MathUtil.applyDeadband(
                              Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()),
                              DEADBAND);

                      double omega;
                      if (drive.hasNoteTarget()) {
                        omega = pidController.calculate(drive.getNoteLocation(), 0);
                        Logger.recordOutput("NoteVision/PIDCalculated", omega);
                      } else {
                        omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);
                        Logger.recordOutput("NoteVision/PIDCalculated", 0);
                      }

                      // Square values
                      linearMagnitude = linearMagnitude * linearMagnitude;

                      drive.runVelocity(
                          new ChassisSpeeds(
                              -linearMagnitude * drive.getMaxLinearSpeedMetersPerSec(), 0, omega));
                    },
                    drive),
                Commands.waitUntil(
                    () ->
                        Math.abs(drive.getLastNoteLocation() - drive.getNoteLocation())
                                > Constants.IntakeConstants.noteLockTolerance
                            && drive.lastHasNoteTarget())),
            Commands.run(
                () -> {
                  // Apply deadband
                  double linearMagnitude =
                      MathUtil.applyDeadband(
                          Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);

                  // Square values
                  linearMagnitude = linearMagnitude * linearMagnitude;

                  Logger.recordOutput("NoteVision/PIDCalculated", 0);

                  drive.runVelocity(
                      new ChassisSpeeds(
                          -linearMagnitude * drive.getMaxLinearSpeedMetersPerSec(), 0, 0));
                },
                drive));

    return driveTrainCommand.alongWith(indexer.getDefault(pivot::inHandoffZone));
  }

  public static Command autoAimPivot(Drive drive, Pivot pivot) {
    DoubleSupplier distance = () -> getSpeaker().getDistance(drive.getPose().getTranslation());
    return pivot.setPositionCommand(() -> interpolator.get(distance.getAsDouble()));
  }
}
