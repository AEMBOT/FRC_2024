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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final DigitalInput robotJumper = new DigitalInput(0);
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
  public static final Robot currentRobot = robotJumper.get() ? Robot.CLEF : Robot.LIGHTCYCLE;

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public enum Robot {
    CLEF,
    LIGHTCYCLE
  }

  public static final class ClimberConstants {
    public static final int winchMotorRightCanID = 15;
    public static final int winchMotorLeftCanID = 14;
    public static final int homingCurrentLimit = 40;
    public static final int extendCurrentLimit = 60;
    public static final double extendToGrab = 10; // change this
    public static final double extendToPullUp = 10; // change this
    public static final double minExtendHardStop = 0;
    public static final double maxExtendSoftStop = 10; // change this
    public static final double extendMetersPerTick = 0.0160734375; // change this lmao
  }

  public static final double UPDATE_PERIOD = 0.02;

  public static final class PivotConstants {
    public static final double PIVOT_MAX_POS_RAD = 2.20;
    public static final double PIVOT_MIN_POS_RAD = 0.30;
  }

  public static final class IndexerConstants {
    /* PORTS */
    public static final int indexerMotorPortBottom = 16;
    public static final int indexerMotorPortTop = 19;
    public static final int indexerBeamBrake = 9;

    /* VOLTAGES */
    public static final double indexerMotorVoltage = 1;
  }

  public static final class IntakeConstants {
    public static final int intakeMotorPortBottom = 13;
    public static final int intakeMotorPortTop = 12;
    public static final int intakeBeamBrake = 7;

    /*VOLTAGES*/
    public static final double intakeMotorVoltage = 12;
  }

  public static final class ShooterConstants {
    public static final double shooterSpeedRPM = 7840;
    public static final double shooterIdleRPM = 980;
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

  public static final class shootingSpeakerConstants {
    public static double kP = 5;
    public static double kI = 0;
    public static double kD = 0.02;
    public static double maxVelocity = 2;
    public static double maxAcceleration = 4;
    public static double[][] shooterInterpolationPoints =
        new double[][] {
          new double[] {1.0, 1.065},
          new double[] {2.0, 0.759},
          new double[] {3.0, 0.550 + 0.02},
          new double[] {3.45, 0.483 + 0.02 + 0.02},
          new double[] {3.70, 0.450 + 0.015 + 0.02},
          new double[] {4.0, 0.471 + 0.02},
          new double[] {4.08, 0.463 + 0.02},
          new double[] {4.85, 0.370 + 0.013 + 0.02},
          new double[] {5.0, 0.360 + 0.01 + 0.02},
          new double[] {6.0, 0.33 + 0.007 + 0.02},
          new double[] {7.0, 0.29 + 0.005 + 0.02}
        };
  }
}
