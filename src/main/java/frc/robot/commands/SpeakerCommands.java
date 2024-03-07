package frc.robot.commands;

import static frc.robot.Constants.FieldConstants.getSpeaker;
import static frc.robot.Constants.shootingSpeakerConstants.*;
import static frc.robot.Constants.shootingSpeakerConstants.noteVelocity;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.pivot.Pivot;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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
        ProfiledPIDController pidController =
                new ProfiledPIDController(
                        kP, kI, kD, new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
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
                new ProxyCommand(pivot.setPositionCommand(() -> interpolator.get(distance.getAsDouble()))));
    }
    public static Command correctionShootSpeaker(
            Drive drive, Pivot pivot, DoubleSupplier xSupplier, DoubleSupplier ySupplier,  Supplier<Pose2d> effectivePose) {
        DoubleSupplier distance = () -> getSpeaker().getDistance(effectivePose.get().getTranslation());
        ProfiledPIDController pidController =
                new ProfiledPIDController(
                        kP, kI, kD, new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
        pidController.enableContinuousInput(-Math.PI, Math.PI);
        Command driveTrainCommand =
                Commands.run(
                        () -> {
                            Rotation2d targetAngle =
                                    getSpeaker().minus(effectivePose.get().getTranslation()).getAngle();
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
                new ProxyCommand(pivot.setPositionCommand(() -> interpolator.get(distance.getAsDouble()))));
    }
    public static Command intakeNote(
            Drive drive,
            Indexer indexer,
            Pivot pivot,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier) {
        PIDController pidController = new PIDController(1, 0, 0);
        pidController.enableContinuousInput(-Math.PI, Math.PI);
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
                            } else {
                                omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);
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
                                            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                                            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                                            omega,
                                            isFlipped // Is this necessary? test on field
                                                    ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                                    : drive.getRotation()));
                        },
                        drive);
        return driveTrainCommand.alongWith(indexer.getDefault(pivot::inHandoffZone));
    }

    public static Pose2d getAdjustedPose(Drive drive){
        double distance = getSpeaker().getDistance(drive.getPose().getTranslation());
        Rotation2d rotation = getSpeaker().minus(drive.getPose().getTranslation()).getAngle();
        Translation2d adjustedVelocity = new Translation2d(drive.getVelocity().vxMetersPerSecond,drive.getVelocity().vyMetersPerSecond).rotateBy(rotation.unaryMinus());
        Translation2d adjustedNoteVelocity = new Translation2d(Math.sqrt(noteVelocity*noteVelocity - adjustedVelocity.getY()*adjustedVelocity.getY()), -adjustedVelocity.getY());
        double flightTime = distance/(adjustedNoteVelocity.getX() + adjustedVelocity.getX());
        Translation2d adjustedPosition = new Translation2d(distance, 0).plus(adjustedVelocity.times(flightTime));
        Translation2d realPosition = adjustedPosition.minus(getSpeaker()).rotateBy(rotation).plus(getSpeaker());
        return new Pose2d(realPosition, drive.getPose().getRotation());
    }
    public static Command shootMove(Drive drive, Pivot pivot, DoubleSupplier xSupplier, DoubleSupplier ySupplier){
        return correctionShootSpeaker(drive, pivot, xSupplier, ySupplier, () -> getAdjustedPose(drive));
    }
}
