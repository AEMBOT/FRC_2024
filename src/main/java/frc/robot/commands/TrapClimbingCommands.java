package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward;

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
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.FieldConstants;

public class TrapClimbingCommands {
 //center with vision
 //bring pivot down (110 degrees)
    //bring climber up
 //drive forward until x distance away (read using prev april tags and odom)
 //bring climber down
 //once thats done sequential command for bring pivot down

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
                        ? FieldConstants.RED_TRAP_CHAIN_POSE
                        : FieldConstants.BLUE_TRAP_CHAIN_POSE) //center drivebase to trap
                .andThen(
                new WaitCommand(0.5) //wait .5 seconds after, only for testing
                .andThen(
                    drive.pathFindingCommand(
                        isFlipped
                        ?FieldConstants.RED_TRAP_CENTER_POSE
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
}