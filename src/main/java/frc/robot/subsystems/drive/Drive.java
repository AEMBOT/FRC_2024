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

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward;
import static edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse;
import static frc.robot.Constants.FieldConstants.getSpeaker;
import static frc.robot.subsystems.drive.Module.WHEEL_RADIUS;
import static java.lang.Math.abs;
import static java.lang.Math.min;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIO;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIOInputsAutoLogged;
import frc.robot.subsystems.notevision.NoteVisionIO;
import frc.robot.subsystems.notevision.NoteVisionIOInputsAutoLogged;
import frc.robot.util.LocalADStarAK;
import java.util.Arrays;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  private static final double MAX_LINEAR_SPEED = Units.feetToMeters(18.5); // MK4i L3+
  private static final double TRACK_WIDTH_X = Units.inchesToMeters(22.75); // 28 in square chassis
  private static final double TRACK_WIDTH_Y = Units.inchesToMeters(22.75);
  public static final double DRIVE_BASE_RADIUS =
      Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
  private static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine moduleSteerRoutine;
  private final SysIdRoutine driveRoutine;

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  private final AprilTagVisionIO aprilTagVisionIO;
  private final AprilTagVisionIOInputsAutoLogged aprilTagVisionInputs =
      new AprilTagVisionIOInputsAutoLogged();

  private final NoteVisionIO noteVisionIO;
  private final NoteVisionIOInputsAutoLogged noteVisionInputs = new NoteVisionIOInputsAutoLogged();

  @AutoLogOutput public boolean useVision = true;

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO,
      AprilTagVisionIO aprilTagVisionIO,
      NoteVisionIO noteVisionIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);
    this.aprilTagVisionIO = aprilTagVisionIO;
    this.noteVisionIO = noteVisionIO;

    // Start threads (no-op for each if no signals have been created)
    PhoenixOdometryThread.getInstance().start();
    SparkMaxOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::setPose,
        () -> kinematics.toChassisSpeeds(getModuleStates()),
        this::runVelocity,
        new HolonomicPathFollowerConfig(
            new PIDConstants(5.0),
            new PIDConstants(5.0),
            MAX_LINEAR_SPEED,
            DRIVE_BASE_RADIUS,
            new ReplanningConfig(true, true)), // PP can't replan choreo paths
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "PathPlanner/ActivePath", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("PathPlanner/Target", targetPose);
          Logger.recordOutput(
              "PathPlanner/AbsoluteTranslationError",
              targetPose.minus(getPose()).getTranslation().getNorm());
        });

    Logger.recordOutput("PathPlanner/Target", new Pose2d());
    Logger.recordOutput("PathPlanner/AbsoluteTranslationError", 0.0);

    // Configure SysId
    moduleSteerRoutine = // I know SignalLogger is CTRE specific but I think this works regardless
        // of IO layer, check
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // Default ramp rate is acceptable
                Volts.of(8),
                null, // Default timeout is acceptable
                // Log state with Phoenix SignalLogger class
                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts) -> modules[0].runSteerCharacterization(volts.in(Volts)),
                null,
                this));
    driveRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // Default ramp rate is acceptable
                Volts.of(6), // Reduce dynamic voltage to 6 to prevent motor brownout
                Seconds.of(20),
                // Log state with Phoenix SignalLogger class
                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts) -> runDriveCharacterizationVolts(volts.in(Volts)),
                null,
                this));
  }

  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    for (var module : modules) {
      module.updateInputs();
    }
    odometryLock.unlock();
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }
    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    // TODO this entire section is completely broken because navx signal timing doesn't match
    // canivore fused timestamps
    // TODO either write custom handler for gyro angle or beg for pigeon 2
    // vision also corrects for async odo so maybe just cope
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int minTimestamps =
        min(
            min(
                modules[0].getOdometryTimestamps().length,
                modules[1].getOdometryTimestamps().length),
            min(
                modules[2].getOdometryTimestamps().length,
                modules[3].getOdometryTimestamps().length)); // Check if one wheel data was rejected
    int sampleCount;
    if (gyroInputs.connected) {
      sampleCount = min(minTimestamps, gyroInputs.odometryYawPositions.length);
    } else {
      sampleCount = minTimestamps;
    }
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    // Update vision
    aprilTagVisionIO.updatePose(getPose());
    aprilTagVisionIO.updateInputs(aprilTagVisionInputs);
    Logger.processInputs("Drive/AprilTagVision", aprilTagVisionInputs);

    for (int i = 0; i < aprilTagVisionInputs.timestamps.length; i++) {
      if ( // Bounds check the pose is actually on the field
      aprilTagVisionInputs.timestamps[i] >= 1.0
          && abs(aprilTagVisionInputs.visionPoses[i].getZ()) < 0.2
          && aprilTagVisionInputs.visionPoses[i].getX() > 0
          && aprilTagVisionInputs.visionPoses[i].getX() < 16.5
          && aprilTagVisionInputs.visionPoses[i].getY() > 0
          && aprilTagVisionInputs.visionPoses[i].getY() < 8.5
          && aprilTagVisionInputs.visionPoses[i].getRotation().getX() < 0.2
          && aprilTagVisionInputs.visionPoses[i].getRotation().getY() < 0.2
      //          && aprilTagVisionInputs
      //                  .visionPoses[i]
      //                  .toPose2d()
      //                  .minus(poseEstimator.getEstimatedPosition())
      //                  .getTranslation()
      //                  .getNorm()
      //              < 3.0 // todo replace this with multi-tag only and no distance cap
      ) {
        Logger.recordOutput(
            "Drive/AprilTagPose" + i, aprilTagVisionInputs.visionPoses[i].toPose2d());
        Logger.recordOutput(
            "Drive/AprilTagStdDevs" + i,
            Arrays.copyOfRange(aprilTagVisionInputs.visionStdDevs, 3 * i, 3 * i + 3));
        Logger.recordOutput("Drive/AprilTagTimestamps" + i, aprilTagVisionInputs.timestamps[i]);

        if (useVision) {
          poseEstimator.addVisionMeasurement(
              aprilTagVisionInputs.visionPoses[i].toPose2d(),
              aprilTagVisionInputs.timestamps[i],
              VecBuilder.fill(
                  aprilTagVisionInputs.visionStdDevs[3 * i],
                  aprilTagVisionInputs.visionStdDevs[3 * i + 1],
                  aprilTagVisionInputs.visionStdDevs[3 * i + 2]));
        }
      } else {
        Logger.recordOutput("Drive/AprilTagPose" + i, new Pose2d());
        Logger.recordOutput("Drive/AprilTagStdDevs" + i, new double[] {0.0, 0.0, 0.0});
      }
    }

    // Update Note Detection
    noteVisionIO.updateInputs(noteVisionInputs);
    Logger.processInputs("Drive/NoteVision", noteVisionInputs);

    Logger.recordOutput(
        "Distance to Speaker", getSpeaker().getDistance(getPose().getTranslation()));
    Logger.recordOutput(
        "Drive/Running Command",
        Optional.ofNullable(this.getCurrentCommand()).map(Command::getName).orElse("None"));
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_LINEAR_SPEED);

    Logger.recordOutput("Swerve/TargetSpeeds", discreteSpeeds);
    Logger.recordOutput("Swerve/SpeedError", discreteSpeeds.minus(getVelocity()));

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public Command runVelocityCommand(Supplier<ChassisSpeeds> speeds) {
    return this.run(() -> runVelocity(speeds.get()));
  }

  /** Stops the drive. */
  public Command stopCommand() {
    return runVelocityCommand(ChassisSpeeds::new);
  }

  public Command runVelocityFieldRelative(Supplier<ChassisSpeeds> speeds) {
    return this.runVelocityCommand(
        () -> ChassisSpeeds.fromFieldRelativeSpeeds(speeds.get(), getRotation()));
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  private void runDriveCharacterizationVolts(double volts) {
    for (Module module : modules) {
      module.runDriveCharacterization(volts);
    }
  }

  /** Returns the average drive velocity in meters/sec. */
  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    for (Module module : modules) {
      driveVelocityAverage += module.getVelocityMetersPerSec();
    }
    return driveVelocityAverage / 4.0;
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive velocities) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  @AutoLogOutput(key = "Odometry/Velocity")
  public ChassisSpeeds getVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        kinematics.toChassisSpeeds(
            Arrays.stream(modules).map(Module::getState).toArray(SwerveModuleState[]::new)),
        getRotation());
  }

  @AutoLogOutput(key = "Odometry/RobotRelativeVelocity")
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(
        (SwerveModuleState[])
            Arrays.stream(modules).map((m) -> m.getState()).toArray(SwerveModuleState[]::new));
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Get the position of all drive wheels in radians. */
  public double[] getWheelRadiusCharacterizationPosition() {
    return Arrays.stream(modules)
        .mapToDouble((module) -> module.getPosition().distanceMeters / WHEEL_RADIUS)
        .toArray();
  }

  /** Gets the raw, unwrapped gyro heading from the gyro. Useful for wheel characterization */
  public double getRawGyroYaw() {
    return gyroInputs.yawPosition.getRadians();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  public void setYaw(Rotation2d yaw) {
    gyroIO.setYaw(yaw);
    setPose(new Pose2d(getPose().getTranslation(), yaw));
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp The timestamp of the vision measurement in seconds.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    poseEstimator.addVisionMeasurement(visionPose, timestamp);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return MAX_LINEAR_SPEED;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return MAX_ANGULAR_SPEED;
  }

  public boolean hasNoteTarget() {
    return noteVisionInputs.hasTarget;
  }

  public double getNoteLocation() {
    return noteVisionInputs.targetX;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)
    };
  }

  public Command runModuleSteerCharacterizationCmd() {
    return Commands.sequence(
        this.runOnce(SignalLogger::start),
        moduleSteerRoutine.quasistatic(kForward),
        this.stopCommand().withTimeout(2.0),
        moduleSteerRoutine.quasistatic(kReverse),
        this.stopCommand().withTimeout(2.0),
        moduleSteerRoutine.dynamic(kForward),
        this.stopCommand().withTimeout(2.0),
        moduleSteerRoutine.dynamic(kReverse),
        this.runOnce(SignalLogger::stop));
  }

  public Command runDriveCharacterizationCmd() { // TODO Timing on this seems sus
    return Commands.sequence(
        this.runOnce(SignalLogger::start),
        driveRoutine.quasistatic(kForward),
        this.stopCommand().withTimeout(2.0),
        driveRoutine.quasistatic(kReverse),
        this.stopCommand().withTimeout(2.0),
        driveRoutine.dynamic(kForward),
        this.stopCommand().withTimeout(2.0),
        driveRoutine.dynamic(kReverse),
        this.runOnce(SignalLogger::stop));
  }

  public void setVisionState(boolean state) {
    useVision = state;
  }
}
