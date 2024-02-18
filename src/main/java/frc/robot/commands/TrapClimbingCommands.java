package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward;

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

public class TrapClimbingCommands {
 //center with vision
 //bring pivot down (110 degrees)
    //bring climber up
 //drive forward until x distance away (read using prev april tags and odom)
 //bring climber down
 //once thats done sequential command for bring pivot down

 public static Command DriveAndClimb(Drive drive, Climber climber, Pivot pivot) {
    
    return Commands.run(
        ()->{
        boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        climber.setPositionCommand(10)
        .alongWith(pivot.setPositionCommand(()-> 110),  
                Commands.run(() -> drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                0, 
                0,
                0,
                drive.getRotation()))).until(null)
                .andThen(
                new WaitCommand(0.5)
                .andThen(
                climber.setPositionCommand(0).andThen(
                pivot.setPositionCommand(()-> 70)
            ))));
        });
        
}
}