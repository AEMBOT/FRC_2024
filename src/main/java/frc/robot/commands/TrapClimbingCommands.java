package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import com.kauailabs.navx.*;

/*FRC Notes for Climbing:
    1. Center to trap/stage using vision
    2. pivot to 120 degree, lift climber
    3. drive under chain until the pivot is touching the stage
    4. drive backwards while pivoting forward until the pivot is 90 degrees
    5. retract climber
    6. lower the pivot angle to 30 degrees */

public class TrapClimbingCommands {
 //center with vision
 //bring pivot down (110 degrees)
    //bring climber up
 //drive forward until x distance away (read using prev april tags and odom)
 //bring climber down
 //once thats done sequential command for bring pivot down

    //just for testing right now
    public static Command DriveFastAndClimb(Drive drive, Climber climber, Pivot pivot, Shooter shooter){
        return Commands.run(
            () -> {
                Pose2d nearestTrapPose2d = new Pose2d(0,0,Rotation2d.fromDegrees(0));
                boolean isFlipped =
                 DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
                if (!isFlipped){
                    List<Pose2d> poseList = new ArrayList<>();
                    poseList.add(Constants.aprilTagFieldLayout.getTagPose(14    ).get().toPose2d());
                    poseList.add(Constants.aprilTagFieldLayout.getTagPose(15).get().toPose2d());
                    poseList.add(Constants.aprilTagFieldLayout.getTagPose(16).get().toPose2d());
                    nearestTrapPose2d = drive.getPose().nearest(poseList);
                }
                else{
                    List<Pose2d> poseList = new ArrayList<>();
                    poseList.add(Constants.aprilTagFieldLayout.getTagPose(11    ).get().toPose2d());
                    poseList.add(Constants.aprilTagFieldLayout.getTagPose(12).get().toPose2d());
                    poseList.add(Constants.aprilTagFieldLayout.getTagPose(13).get().toPose2d());
                    nearestTrapPose2d = drive.getPose().nearest(poseList);
                }
                climber.setPositionCommand(0.75)
                .alongWith(
                    pivot.setPositionCommand(() -> 120),
                    shooter.setVelocityRPMClimberModeCommand(1000)
                )
                .andThen(drive.pathFindingCommand(nearestTrapPose2d)
                )
                .andThen(pivot.setPositionCommand(() -> 90))
                .alongWith(
                    drive.pathFindingCommand(nearestTrapPose2d.plus(Constants.ONE_METER_BACK.times(0.5)))
                )
                .andThen(new WaitCommand(1))
                .andThen(climber.setPositionCommand(0.05))
                .andThen(pivot.setPositionCommand(() ->40));
    });
}

    public static Command DriveAndClimb(Drive drive, Climber climber, Pivot pivot) {
    //TODO: change values for pivot and climber setpoints to realistic positions
        return Commands.run(
        ()->{
        boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        climber.setPositionCommand(10) //climber up
        .alongWith(pivot.setPositionCommand(()-> 110), //parallel pivot to 110 degrees  
        //center to the trap
                Commands.run(
                    () -> 
                drive.pathFindingCommand(
                    isFlipped
                        ? FieldConstants.RED_TRAP_CENTER_POSE
                        : FieldConstants.BLUE_TRAP_CENTER_POSE) //center drivebase to trap
                .andThen(
                new WaitCommand(0.5) //wait .5 seconds after, only for testing
                .andThen(
                    drive.pathFindingCommand(
                        isFlipped
                        ?FieldConstants.RED_TRAP_CHAIN_POSE
                        : FieldConstants.BLUE_TRAP_CHAIN_POSE
                    ) //drivebase translation to chain
                )
                .andThen(
                climber.setPositionCommand(4) //pull down climber after drivebase translation
                .andThen(
                pivot.setPositionCommand(()-> 70) //pull down pivot after climber pull down
            )))));
        });     
    }

    public static Command DriveClimbTrap(Drive drive, Climber climber, Pivot pivot, Shooter shooter){
         return Commands.run(
        ()->{
        boolean trapShooterCondition = true; //get proper navx data
        boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        climber.setPositionCommand(10) //climber up
        .alongWith(pivot.setPositionCommand(()-> 110), //parallel pivot to 110 degrees  
                Commands.run(
                    () -> 
                drive.pathFindingCommand(
                    isFlipped
                        ? FieldConstants.RED_TRAP_CENTER_POSE
                        : FieldConstants.BLUE_TRAP_CENTER_POSE) //center drivebase to trap
                .andThen(
                    drive.pathFindingCommand(
                        isFlipped
                        ?FieldConstants.RED_TRAP_CHAIN_POSE
                        : FieldConstants.BLUE_TRAP_CHAIN_POSE
                    ) //drivebase translation to chain
                )
                .andThen(
                climber.setPositionCommand(4)) //pull down climber after drivebase translation
                .alongWith(
                pivot.setPositionCommand(() -> 100)).onlyIf(() -> trapShooterCondition)
                .andThen(
                shooter.setVelocityRPMCommand(3000)).onlyIf(() -> trapShooterCondition)
                .andThen(
                shooter.setVelocityRPMCommand(0))
                .andThen(
                pivot.setPositionCommand(()-> 70)) //pull down pivot after climber pull down
            ));
        });     
    }
}