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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.SIM;

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class ClimberConstants {
    public static final int winchMotorRightCanID = 10; // change this
    public static final int winchMotorLeftCanID = 11; // change this
    public static final int homingCurrentLimit = 20;
    public static final int extendCurrentLimit = 60;
    public static final double extendToGrab = 10; // change this
    public static final double extendToPullUp = 10; // change this
    public static final double minExtendHardStop = 0;
    public static final double maxExtendSoftStop = 10; // change this
    public static final double extendMetersPerTick = 0.0160734375; // change this lmao
  }

  public static final double UPDATE_PERIOD = 0.02;

  public final class PivotConstants {
    public static final double PIVOT_MAX_POS_RAD = 0;
    public static final double PIVOT_MIN_POS_RAD = 0;
  }

  public static final class IndexerConstants {
    /* PORTS */
    public static final int indexerMotorPortBottom = 0; // PLACEHOLDER VALUE
    public static final int indexerMotorPortTop = 1; // PLACEHOLDER VALUE
    public static final int indexerBeamBrake = 1; // PLACEHOLDER VALUE

    /* VOLTAGES */
    public static final double indexerMotorVoltage = 1; // PLACEHOLDER VALUE
  }

  public static final class IntakeConstants {
    public static final int intakeMotorPortBottom = 2; // PLACEHOLDER VALUE
    public static final int intakeMotorPortTop = 3; // PLACEHOLDER VALUE
    public static final int intakeBeamBrake = 2; // PLACEHOLDER VALUE

    /*VOLTAGES*/
    public static final double intakeMotorVoltage = 1; // PLACEHOLDER VALUE
  }

  public static final class ShooterConstants {
    public static final double shooterSpeedRPM = 7840;
    public static final double shooterIdleRPM = 1960;
  }

  public static final AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  public final class FieldConstants {
    public static final Translation2d BLUE_SPEAKER_POSE = new Translation2d(-0.086473, 5.757474);
    public static final Translation2d RED_SPEAKER_POSE = new Translation2d(16.389722, 5.757474);

    public static Translation2d getSpeaker() {
      if (DriverStation.getAlliance().isPresent()) {
        return DriverStation.getAlliance().get() == DriverStation.Alliance.Red
            ? RED_SPEAKER_POSE
            : BLUE_SPEAKER_POSE;
      } else {
        return BLUE_SPEAKER_POSE; // default to blue
      }
    }
  }
}
