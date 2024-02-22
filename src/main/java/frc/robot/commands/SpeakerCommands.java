package frc.robot.commands;

import static frc.robot.Constants.FieldConstants.getSpeaker;
import static frc.robot.Constants.shootingSpeakerConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.pivot.Pivot;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class SpeakerCommands {
  private static final double DEADBAND = 0.1;

  private static final InterpolatingDoubleTreeMap interpolator = new InterpolatingDoubleTreeMap();

  static {
    for (double[] shooterInterpolationPoint : shooterInterpolationPoints) {
      interpolator.put(shooterInterpolationPoint[0], shooterInterpolationPoint[1]);
    }
  }

  public static Command shootSpeaker(
      Drive drive, Pivot pivot, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    DoubleSupplier distance = () -> getSpeaker().getDistance(drive.getPose().getTranslation());
    ProfiledPIDController pidController =
        new ProfiledPIDController(
            kP, kI, kD, new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
    pidController.enableContinuousInput(0, 2 * Math.PI);
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
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega,
                      isFlipped // Is this necessary? test on field
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));

              Logger.recordOutput(
                  "Interpolator Distance", interpolator.get(distance.getAsDouble()));
            },
            drive);

    return driveTrainCommand.alongWith(
        pivot.setPositionCommand(() -> interpolator.get(distance.getAsDouble())));
  }
}
