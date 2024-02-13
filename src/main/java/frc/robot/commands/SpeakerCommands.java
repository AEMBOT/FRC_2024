package frc.robot.commands;

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
import static frc.robot.Constants.shootingSpeakerConstants.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;


public class SpeakerCommands {
    private static final double DEADBAND = 0.1;
    private static double interpolation(double distance){
        InterpolatingDoubleTreeMap interpolator = new InterpolatingDoubleTreeMap();
        for(int i=0; i<=shooterInterpolationPoints.length; i++){
            interpolator.put(shooterInterpolationPoints[i][0], shooterInterpolationPoints[i][1]);
        }
        return interpolator.get(distance);
    }
    public static Command shootSpeaker(
            Drive drive,
            Pivot pivot,
            Supplier<Pose2d> robotPosition,
            Translation2d speakerPosition,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier
            ){
        double distance = robotPosition.get().getTranslation().getDistance(speakerPosition);
        Rotation2d targetAngle = speakerPosition.minus(robotPosition.get().getTranslation()).getAngle();
        ProfiledPIDController pidController = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
        Command driveTrainCommand = Commands.run(
                () -> {
                    // Apply deadband
                    double linearMagnitude =
                            MathUtil.applyDeadband(
                                    Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
                    Rotation2d linearDirection =
                            new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
                    double omega = MathUtil.applyDeadband(pidController.calculate(drive.getRotation().getRadians(),targetAngle.getRadians()), DEADBAND);

                    // Square values
                    linearMagnitude = linearMagnitude * linearMagnitude;
                    omega = Math.copySign(omega * omega, omega);

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
                                    omega * drive.getMaxAngularSpeedRadPerSec(),
                                    isFlipped
                                            ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                            : drive.getRotation()));
                },
                drive);

        return driveTrainCommand.alongWith(pivot.goToAngle(interpolation(distance)));
    }
}
