package frc.robot.commands;

import com.kauailabs.navx.*;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import java.util.ArrayList;
import java.util.List;

/*FRC Notes for Climbing:
1. Center to trap/stage using vision
2. pivot to 120 degree, lift climber
3. drive under chain until the pivot is touching the stage
4. drive backwards while pivoting forward until the pivot is 90 degrees
5. retract climber
6. lower the pivot angle to 30 degrees */

public class TrapClimbingCommands {
  // center with vision
  // bring pivot down (110 degrees)
  // bring climber up
  // drive forward until x distance away (read using prev april tags and odom)
  // bring climber down
  // once thats done sequential command for bring pivot down

  // just for testing right now
  public static Command DriveFastAndClimb(
      Drive drive, Climber climber, Pivot pivot, Shooter shooter) {
    Pose2d nearestTrapPose2d = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    if (!isFlipped) {
      List<Pose2d> poseList = new ArrayList<>();
      poseList.add(
          Constants.aprilTagFieldLayout
              .getTagPose(14)
              .get()
              .toPose2d()
              .plus(Constants.ONE_METER_BACK.times(0.25)));
      poseList.add(
          Constants.aprilTagFieldLayout
              .getTagPose(15)
              .get()
              .toPose2d()
              .plus(Constants.ONE_METER_BACK.times(0.25)));
      poseList.add(
          Constants.aprilTagFieldLayout
              .getTagPose(16)
              .get()
              .toPose2d()
              .plus(Constants.ONE_METER_BACK.times(0.25)));
      nearestTrapPose2d = drive.getPose().nearest(poseList);
    } else {
      List<Pose2d> poseList = new ArrayList<>();
      poseList.add(
          Constants.aprilTagFieldLayout
              .getTagPose(11)
              .get()
              .toPose2d()
              .plus(Constants.ONE_METER_BACK.times(0.25)));
      poseList.add(
          Constants.aprilTagFieldLayout
              .getTagPose(12)
              .get()
              .toPose2d()
              .plus(Constants.ONE_METER_BACK.times(0.2)));
      poseList.add(
          Constants.aprilTagFieldLayout
              .getTagPose(13)
              .get()
              .toPose2d()
              .plus(Constants.ONE_METER_BACK.times(0.25)));
      nearestTrapPose2d = drive.getPose().nearest(poseList);
    }
    return Commands.parallel(
            climber.setPositionCommand(0.75),
            pivot.setPositionCommand(() -> 2.0),
            // shooter.setVelocityRPMClimberModeCommand(1000),
            drive.pathFindingCommand(nearestTrapPose2d))
        .until(() -> pivot.atGoal() && climber.atGoal())
        .andThen(
            Commands.parallel(
                    pivot.setPositionCommand(() -> 1.5),
                    drive.pathFindingCommand(
                        nearestTrapPose2d.plus(Constants.ONE_METER_BACK.times(0.5))))
                .until(pivot::atGoal))
        .andThen(new WaitCommand(1))
        .andThen(climber.setPositionCommand(0.05).until(climber::atGoal))
        .andThen(pivot.setPositionCommand(() -> 0.7).until(pivot::atGoal));
  }

  public static Command OTFTesting(Drive drive) {
    Pose2d targetPose =
        Constants.aprilTagFieldLayout
            .getTagPose(12)
            .get()
            .toPose2d()
            .plus(Constants.ONE_METER_BACK.times(0.2));
    return (drive.pathFindingCommand(targetPose));
  }

  public static Command ClimbFirstHalf(Climber climber, Pivot pivot, Shooter shooter) {
    return Commands.parallel(
            climber.setPositionCommand(0.75), pivot.setPositionCommand(() -> 2.0)
            // shooter.setVelocityRPMClimberModeCommand(1000)
            )
        .until(() -> pivot.atGoal() && climber.atGoal());
  }
  /*
  public static Command ClimbSecondHalf(Climber climber, Pivot pivot) {
    return Commands.run(
        () -> {
          pivot
              .setPositionCommand(() -> 70)
              .until(() -> pivot.atGoal())
              .andThen(climber.setPositionCommand(0.05))
              .until(() -> climber.atGoal())
              .andThen(pivot.setPositionCommand(() -> 40))
              .until(() -> pivot.atGoal());
        });
  }*/

  public static Command ClimbSecondHalf(Climber climber, Pivot pivot) {
    return Commands.deadline(
            Commands.waitSeconds(0.2)
                // .andThen(Commands.waitUntil((() -> pivot.atGoal() && climber.atGoal()))),
                .andThen(Commands.waitUntil(() -> climber.atGoal())),
            climber.setPositionCommand(0.10)
            // pivot.setPositionCommand(() -> 1.2))
            )
        .andThen(
            Commands.deadline(
                Commands.waitSeconds(2).andThen(Commands.waitUntil(() -> pivot.atGoal())),
                pivot.setPositionCommand(() -> 0.7)));
  }

  public static Command OTFTestingRev2(Drive drive) {
    Pose2d targetPose =
        Constants.aprilTagFieldLayout
            .getTagPose(12)
            .get()
            .toPose2d()
            .plus(Constants.ONE_METER_BACK.times(0.2));
    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(targetPose);

    PathConstraints constraints =
        new PathConstraints(3, 1, Units.degreesToRadians(100), Units.degreesToRadians(100));

    PathPlannerPath path =
        new PathPlannerPath(
            bezierPoints, constraints, new GoalEndState(0.0, Rotation2d.fromDegrees(0)));
    path.preventFlipping = true;

    return new FollowPathHolonomic(
        path,
        drive::getPose, // Robot pose supplier
        drive::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        drive::runVelocity, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in
            // your Constants class
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            6, // Max module speed, in m/s
            0.4, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options
            // here
            ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        drive // Reference to this subsystem to set requirements
        );

    /*
    Command pathFindCommand = new PathfindThenFollowPathHolonomic(
        path, constraints, drive::getPose, drive::getRobotRelativeSpeeds, drive::runVelocityFieldRelative,
        new HolonomicPathFollowerConfig(
                        new PIDConstants(5.0, 0.0, 0.0),
                        new PIDConstants(5.0, 0.0, 0.0),
                        4.5,
                        0.4,
                        new ReplanningConfig()
                ),
    3.0,
    () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    },
    drive
    );*/
  }

  /*
  public static Command DriveAndClimb(Drive drive, Climber climber, Pivot pivot) {
    // TODO: change values for pivot and climber setpoints to realistic positions
    return Commands.run(
        () -> {
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
          climber
              .setPositionCommand(10) // climber up
              .alongWith(
                  pivot.setPositionCommand(() -> 110), // parallel pivot to 110 degrees
                  // center to the trap
                  Commands.run(
                      () ->
                          drive
                              .pathFindingCommand(
                                  isFlipped
                                      ? FieldConstants.RED_TRAP_CENTER_POSE
                                      : FieldConstants
                                          .BLUE_TRAP_CENTER_POSE) // center drivebase to trap
                              .andThen(
                                  new WaitCommand(0.5) // wait .5 seconds after, only for testing
                                      .andThen(
                                          drive.pathFindingCommand(
                                              isFlipped
                                                  ? FieldConstants.RED_TRAP_CHAIN_POSE
                                                  : FieldConstants
                                                      .BLUE_TRAP_CHAIN_POSE) // drivebase
                                          // translation to chain
                                          )
                                      .andThen(
                                          climber
                                              .setPositionCommand(
                                                  4) // pull down climber after drivebase
                                              // translation
                                              .andThen(
                                                  pivot.setPositionCommand(
                                                      () ->
                                                          70) // pull down pivot after climber pull
                                                  // down
                                                  )))));
        });
  }

  public static Command DriveClimbTrap(Drive drive, Climber climber, Pivot pivot, Shooter shooter) {
    return Commands.run(
        () -> {
          boolean trapShooterCondition = true; // get proper navx data
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
          climber
              .setPositionCommand(10) // climber up
              .alongWith(
                  pivot.setPositionCommand(() -> 110), // parallel pivot to 110 degrees
                  Commands.run(
                      () ->
                          drive
                              .pathFindingCommand(
                                  isFlipped
                                      ? FieldConstants.RED_TRAP_CENTER_POSE
                                      : FieldConstants
                                          .BLUE_TRAP_CENTER_POSE) // center drivebase to trap
                              .andThen(
                                  drive.pathFindingCommand(
                                      isFlipped
                                          ? FieldConstants.RED_TRAP_CHAIN_POSE
                                          : FieldConstants
                                              .BLUE_TRAP_CHAIN_POSE) // drivebase translation to
                                  // chain
                                  )
                              .andThen(
                                  climber.setPositionCommand(
                                      4)) // pull down climber after drivebase translation
                              .alongWith(pivot.setPositionCommand(() -> 100))
                              .onlyIf(() -> trapShooterCondition)
                              .andThen(shooter.setVelocityRPMCommand(3000))
                              .onlyIf(() -> trapShooterCondition)
                              .andThen(shooter.setVelocityRPMCommand(0))
                              .andThen(
                                  pivot.setPositionCommand(
                                      () -> 70)) // pull down pivot after climber pull down
                      ));
        });
  }*/
}
