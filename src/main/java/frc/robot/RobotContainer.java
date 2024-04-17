// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.ShooterConstants.shooterSpeedRPM;
import static frc.robot.commands.SpeakerCommands.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.WheelRadiusCharacterization;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIO;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIOReal;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIOSim;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberIOSparkMax;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOSparkMax;
import frc.robot.subsystems.notevision.NoteVisionIO;
import frc.robot.subsystems.notevision.NoteVisionIOReal;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOReal;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  public final Indexer indexer;
  public final Pivot pivot;
  public final Shooter shooter;
  private final Climber climber;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController backupController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // LEDs
  private final SerialPort prettyLights = new SerialPort(115200, SerialPort.Port.kMXP);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOTalonFX(0),
                new ModuleIOTalonFX(1),
                new ModuleIOTalonFX(2),
                new ModuleIOTalonFX(3),
                new AprilTagVisionIOReal(),
                new NoteVisionIOReal());
        indexer = new Indexer(new IndexerIOSparkMax());
        pivot = new Pivot(new PivotIOReal());
        shooter = new Shooter(new ShooterIOReal());
        climber = new Climber(new ClimberIOSparkMax());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new AprilTagVisionIOSim(),
                new NoteVisionIO() {});
        indexer = new Indexer(new IndexerIOSim());
        pivot = new Pivot(new PivotIOSim());
        shooter = new Shooter(new ShooterIOSim());
        climber = new Climber(new ClimberIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new AprilTagVisionIO() {},
                new NoteVisionIO() {});
        indexer = new Indexer(new IndexerIO() {});
        pivot = new Pivot(new PivotIO() {});
        shooter = new Shooter(new ShooterIO() {});
        climber = new Climber(new ClimberIO() {});
        break;
    }

    // Set up auto commands
    NamedCommands.registerCommand(
        "shootNoteSubwoofer",
        Commands.sequence(
            runOnce(() -> Logger.recordOutput("autoState", -1)),
            Commands.deadline(
                Commands.sequence(
                    runOnce(() -> Logger.recordOutput("autoState", 0.05)),
                    waitSeconds(0.2),
                    runOnce(() -> Logger.recordOutput("autoState", 0.1)),
                    waitUntil(() -> pivot.atGoal() && shooter.isAtShootSpeed()).withTimeout(0.3),
                    runOnce(() -> Logger.recordOutput("autoState", 0.2)),
                    new ProxyCommand(indexer.shootCommand().withTimeout(0.3).withName("shoot"))),
                Commands.sequence(
                    runOnce(() -> Logger.recordOutput("autoState", 0.02)),
                    new ProxyCommand(
                        pivot
                            .setPositionCommand(() -> Units.degreesToRadians(60))
                            .withName("sub"))),
                Commands.sequence(
                    runOnce(() -> Logger.recordOutput("autoState", 0.04)),
                    new ProxyCommand(
                        shooter.setVelocityRPMCommand(shooterSpeedRPM).withName("shoot")))),
            runOnce(() -> Logger.recordOutput("autoState", 0.3)),
            Commands.sequence(runOnce(() -> Logger.recordOutput("autoState", 0.4)))));
    //    NamedCommands.registerCommand(
    //        "shootNoteAuto",
    //        Commands.deadline(
    //            waitSeconds(0.2)
    //                .andThen(waitUntil(() -> pivot.atGoal() && shooter.isAtShootSpeed()))
    //                .andThen(new ProxyCommand(indexer.shootCommand().withTimeout(0.3))),
    //            new ProxyCommand(
    //                pivot.setPositionCommand(
    //                    () ->
    //                        interpolator.get(
    //                            getSpeaker().getDistance(drive.getPose().getTranslation())))),
    //            new ProxyCommand(shooter.setVelocityRPMCommand(shooterSpeedRPM))));
    NamedCommands.registerCommand(
        "shootNoteAuto",
        Commands.deadline(
            waitSeconds(0.04)
                .andThen(waitUntil(() -> pivot.atGoal() && shooter.isAtShootSpeed()))
                .andThen(new ProxyCommand(indexer.shootCommand().withTimeout(0.3))),
            shootSpeaker(drive, pivot, () -> 0, () -> 0),
            new ProxyCommand(
                shooter.setVelocityRPMCommand(shooterSpeedRPM).withName("shootNoteAuto Fire"))));
    //    NamedCommands.registerCommand(
    //        "intakeNote", indexer.getDefault(pivot::inHandoffZone).withTimeout(3.0));
    NamedCommands.registerCommand(
        "intakeNote", new InstantCommand(() -> Logger.recordOutput("Intake Scheduled", true)));

    NamedCommands.registerCommand(
        "spinUpShooter",
        new ProxyCommand(shooter.setVelocityRPMCommand(shooterSpeedRPM).withName("Pre-Spinup")));
    NamedCommands.registerCommand("autoAimPivot", new ProxyCommand(autoAimPivot(drive, pivot)));

    // Set up Auto Routines
    NamedCommands.registerCommand(
        "Nine Piece Auto",
        runOnce(
                () ->
                    drive.setPose(
                        PathPlannerPath.fromChoreoTrajectory("ninepieceauto")
                            .getTrajectory(new ChassisSpeeds(), new Rotation2d())
                            .getInitialTargetHolonomicPose()))
            .andThen(
                AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("ninepieceauto"))));
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    autoChooser.addOption("Nine Piece Auto", NamedCommands.getCommand("Nine Piece Auto"));
    // Set up SysId routines
    autoChooser.addOption("Swerve Drive SysId Routine", drive.runDriveCharacterizationCmd());
    autoChooser.addOption("Swerve Steer SysId Routine", drive.runModuleSteerCharacterizationCmd());
    autoChooser.addOption(
        "Score Preload",
        pivot
            .setPositionCommand(() -> Units.degreesToRadians(60))
            .alongWith(
                shooter
                    .setVelocityRPMCommand(shooterSpeedRPM)
                    .alongWith(
                        Commands.waitUntil(shooter::isAtShootSpeed)
                            .andThen(indexer.shootCommand())))
            .withTimeout(2.0));

    // Configure the button bindings
    configureButtonBindings();

    // Configure Light Bindings
    prettyLights.writeString("l");
    configureLightBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX(),
            () -> controller.getLeftTriggerAxis() > 0.5)); // Trigger locks make trigger 0/1
    indexer.setDefaultCommand(
        indexer.getDefault(pivot::inHandoffZone).withName("Indexer Auto Default Run"));
    pivot.setDefaultCommand(pivot.getDefault());
    shooter.setDefaultCommand(shooter.getDefault());

    // Subwoofer
    controller
        .b()
        .whileTrue(
            pivot
                .setPositionCommand(() -> Units.degreesToRadians(60))
                .alongWith(drive.stopWithXCommand()));
    // Trap
    controller.y().whileTrue(pivot.setPositionCommand(() -> 1.81));
    // Return to Stow
    //    controller.x().whileTrue(pivot.setPositionCommand(() -> 0.44));

    // Pivot Manual Up
    controller
        .povUp()
        .whileTrue(pivot.changeGoalPosition(0.5))
        .onFalse(pivot.changeGoalPosition(0.0));
    controller
        .povDown()
        .whileTrue(pivot.changeGoalPosition(-0.5))
        .onFalse(pivot.changeGoalPosition(0.0));

    // Intake Manual In
    controller.x().whileTrue(indexer.getDefault(pivot::inHandoffZone));
    // Intake Note Vision In With Vector Lock
    controller
        .rightBumper()
        .whileTrue(
            intakeNoteVectorLock(
                drive,
                indexer,
                pivot,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX()));
    // "Intake Out" - Indexer Manual Run
    controller
        .leftBumper()
        .whileTrue(
            shooter
                .setVoltageCommand(1.0)
                .alongWith(Commands.waitSeconds(0.05).andThen(indexer.ampCommand())));

    // Auto Rotation Lock Shooter Pivot Interp
    controller
        .a()
        .whileTrue(
            shootSpeaker(drive, pivot, () -> -controller.getLeftY(), () -> -controller.getLeftX())
                .alongWith(new ProxyCommand(shooter.setVelocityRPMCommand(shooterSpeedRPM))));

    // Z, climb up
    controller.button(10).whileTrue(climber.setPositionCommand(0.75));
    controller.button(10).onTrue(pivot.setPositionCommand(() -> Units.degreesToRadians(120)));
    controller.button(10).onFalse(pivot.setPositionCommand(() -> Units.degreesToRadians(90)));
    // C, climb down
    controller.button(9).whileTrue(climber.setPositionCommand(0.05));

    controller
        .rightTrigger()
        .debounce(0.5, Debouncer.DebounceType.kFalling)
        .whileTrue(
            shooter
                .setVelocityRPMCommand(shooterSpeedRPM)
                .alongWith(
                    Commands.waitUntil(shooter::isAtShootSpeed).andThen(indexer.shootCommand())));

    controller.start().onTrue(runOnce(() -> drive.setYaw(new Rotation2d())).ignoringDisable(true));
    controller.back().onTrue(runOnce(() -> drive.setPose(new Pose2d(7.0, 4.0, new Rotation2d()))));

    new Trigger(indexer::intakedNote)
        .onTrue(
            Commands.run(
                    () -> controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.5))
                .withTimeout(0.2)
                .andThen(
                    Commands.runOnce(
                        () ->
                            controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0)))
                .ignoringDisable(true));

    // Intake Indexer Backwards Eject
    backupController.b().whileTrue(indexer.run(indexer::intakeIndexBackwards));
    // Climb Manual Up
    backupController.povRight().whileTrue(climber.runVoltsCommand(6.0));
    // Climb Manual Down
    backupController.povLeft().whileTrue(climber.runVoltsCommand(-6.0));

    backupController.x().whileTrue(run(() -> drive.setVisionState(false)));
    backupController
        .leftTrigger()
        .whileTrue(
            new WheelRadiusCharacterization(
                drive, WheelRadiusCharacterization.Direction.COUNTER_CLOCKWISE));

    backupController.povDown().whileTrue(climber.getHomingCommand());

    backupController.y().whileTrue(shooter.setVelocityRPMCommand(8000, 0));
    backupController.leftBumper().whileTrue(shooter.setVelocityRPMCommand(0, 8000));
  }

  public void configureLightBindings() {
    Runnable resetColorToIdle =
        () -> {
          if (DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            prettyLights.writeString("r");
          } else if (DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            prettyLights.writeString("b");
          }
        };

    new Trigger(() -> DriverStation.getAlliance().isPresent())
        .onTrue(Commands.runOnce(resetColorToIdle).ignoringDisable(true));

    //    new Trigger(indexer::intakedNote)
    //        .debounce(2.0, Debouncer.DebounceType.kFalling)
    //        .whileTrue(
    //            Commands.startEnd(() -> prettyLights.writeString("g"), resetColorToIdle)
    //                .ignoringDisable(true));

    new Trigger(indexer::intakedNote).onTrue(Commands.runOnce(() -> prettyLights.writeString("g")));
    new Trigger(indexer::hasNote)
        .debounce(2.0, Debouncer.DebounceType.kFalling)
        .onFalse(Commands.runOnce(resetColorToIdle));

    controller
        .rightTrigger()
        .debounce(2.0, Debouncer.DebounceType.kFalling)
        .whileTrue(
            Commands.sequence(
                    Commands.startEnd(() -> prettyLights.writeString("s"), resetColorToIdle))
                .ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  @AutoLogOutput
  public Pose3d[] log3DPoses() {
    Pose3d[] mechanismPoses = new Pose3d[1];
    mechanismPoses[0] = pivot.getPose3D();
    return mechanismPoses;
  }

  public void homeClimber() {
    CommandScheduler.getInstance().schedule(climber.getHomingCommand().withTimeout(10.0));
  }
}
