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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIO;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIOReal;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIOSim;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOSparkMax;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Indexer indexer;
  private final Pivot pivot;
  private final Shooter shooter;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardNumber flywheelSpeedInput =
      new LoggedDashboardNumber("Flywheel Speed", 1500.0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOAlpha(0),
                new ModuleIOAlpha(1),
                new ModuleIOAlpha(2),
                new ModuleIOAlpha(3),
                new AprilTagVisionIOReal());
        indexer = new Indexer(new IndexerIOSparkMax());
        pivot = new Pivot(new PivotIO() {}); // TODO real pivot impl
        shooter = new Shooter(new ShooterIOReal());
        // drive = new Drive(
        // new GyroIOPigeon2(true),
        // new ModuleIOTalonFX(0),
        // new ModuleIOTalonFX(1),
        // new ModuleIOTalonFX(2),
        // new ModuleIOTalonFX(3));
        // flywheel = new Flywheel(new FlywheelIOTalonFX());
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
                new AprilTagVisionIOSim());
        indexer = new Indexer(new IndexerIOSim());
        pivot = new Pivot(new PivotIOSim());
        shooter = new Shooter(new ShooterIOSim());
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
                new AprilTagVisionIO() {});
        indexer = new Indexer(new IndexerIO() {});
        pivot = new Pivot(new PivotIO() {});
        shooter = new Shooter(new ShooterIO() {});
        break;
    }

    // Set up auto routines
    NamedCommands.registerCommand(
        "Nine Piece Auto",
        Commands.runOnce(
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

    // Configure the button bindings
    configureButtonBindings();
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
            () -> -controller.getRightX()));
    indexer.setDefaultCommand(indexer.getDefaultCommand());

    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
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
}
