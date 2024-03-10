package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;

public class ClimbingCommandsRev2 {
  public static Command ClimbFirstHalfRev2(Climber climber, Shooter shooter) {
    return Commands.parallel(
            climber.setPositiionCommandClimbing(0.75),
            // pivot.setPositionCommand(() -> 2.0),
            shooter.setVelocityRPMClimberModeCommand(1000))
        .until(() -> climber.atGoal());
  }

  public static Command ClimbSeconHalfRev2(Climber climber, Pivot pivot) {
    return Commands.deadline(
            Commands.waitSeconds(0.2)
                // .andThen(Commands.waitUntil((() -> pivot.atGoal() && climber.atGoal()))),
                .andThen(Commands.waitUntil(() -> climber.atGoal())),
            climber.setPositiionCommandClimbing(0.15)
            // pivot.setPositionCommand(() -> 1.2))
            )
        .andThen(
            Commands.deadline(
                Commands.waitSeconds(2).andThen(Commands.waitUntil(() -> pivot.atGoal())),
                pivot.setPositionCommand(() -> 0.7)));
  }
}
