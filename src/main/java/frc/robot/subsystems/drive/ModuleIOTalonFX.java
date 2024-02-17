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

import static frc.robot.subsystems.drive.Module.WHEEL_RADIUS;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.Queue;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder. Implementation applicable for 2024 Lightcycle and 2024 Clef.
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using an analog encoder, copy from "ModuleIOSparkMax")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOTalonFX implements ModuleIO {
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder cancoder;

  private final Queue<Double> timestampQueue;

  private final StatusSignal<Double> drivePosition;
  private final Queue<Double> drivePositionQueue;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveCurrent;

  private final StatusSignal<Double> turnAbsolutePosition;
  private final StatusSignal<Double> turnPosition;
  private final Queue<Double> turnPositionQueue;
  private final StatusSignal<Double> turnVelocity;
  private final StatusSignal<Double> turnAppliedVolts;
  private final StatusSignal<Double> turnCurrent;

  // Gear ratios for SDS MK4i L2+, adjust as necessary
  private final double DRIVE_GEAR_RATIO = (50.0 / 16.0) * (17.0 / 27.0) * (45.0 / 15.0);
  private final double TURN_GEAR_RATIO = 150.0 / 7.0;

  private final boolean isDriveMotorInverted;
  private final boolean isTurnMotorInverted;
  private final Rotation2d absoluteEncoderOffset;

  // Control Modes
  private final VoltageOut driveVoltage = new VoltageOut(0.0).withEnableFOC(true);
  private final VoltageOut turnVoltage = new VoltageOut(0.0).withEnableFOC(true);
  private final VelocityVoltage drivePIDF = new VelocityVoltage(0.0).withEnableFOC(true);
  private final PositionVoltage turnPID = new PositionVoltage(0.0).withEnableFOC(true);

  public ModuleIOTalonFX(int index) {
    switch (index) {
      case 0:
        driveTalon = new TalonFX(7, "*");
        turnTalon = new TalonFX(8, "*");
        cancoder = new CANcoder(26, "*");
        absoluteEncoderOffset = Rotation2d.fromRadians(0.32827); // MUST BE CALIBRATED
        isDriveMotorInverted = false;
        isTurnMotorInverted = true;
        break;
      case 1:
        driveTalon = new TalonFX(5, "*");
        turnTalon = new TalonFX(6, "*");
        cancoder = new CANcoder(24, "*");
        absoluteEncoderOffset = Rotation2d.fromRadians(-0.65654 + Math.PI); // MUST BE CALIBRATED
        isDriveMotorInverted = false;
        isTurnMotorInverted = true;
        break;
      case 2:
        driveTalon = new TalonFX(3, "*");
        turnTalon = new TalonFX(4, "*");
        cancoder = new CANcoder(25, "*");
        absoluteEncoderOffset = Rotation2d.fromRadians(-1.41586); // MUST BE CALIBRATED
        isDriveMotorInverted = false;
        isTurnMotorInverted = false;
        break;
      case 3:
        driveTalon = new TalonFX(9, "*");
        turnTalon = new TalonFX(2, "*");
        cancoder = new CANcoder(23, "*");
        absoluteEncoderOffset = Rotation2d.fromRadians(-1.29468 + Math.PI); // MUST BE CALIBRATED
        isDriveMotorInverted = true;
        isTurnMotorInverted = true;
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    // Cancoder
    var cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.MagnetOffset = absoluteEncoderOffset.getRotations();
    cancoder.getConfigurator().apply(cancoderConfig);

    // Drive Configuration
    var driveConfig = new TalonFXConfiguration();

    driveConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    driveConfig.Feedback.SensorToMechanismRatio =
        (DRIVE_GEAR_RATIO) * (1.0 / (WHEEL_RADIUS * 2 * Math.PI));

    driveConfig.Slot0.kV = 2.28;
    driveConfig.Slot0.kA = 0.08;
    driveConfig.Slot0.kS = 0.25;
    driveConfig.Slot0.kP = 2.5; // TODO hand tune
    driveConfig.Slot0.kD = 0.0;

    driveTalon.getConfigurator().apply(driveConfig);
    setDriveBrakeMode(true);

    // Turn Configuration
    var turnConfig = new TalonFXConfiguration();

    turnConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    turnConfig.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
    turnConfig.Feedback.RotorToSensorRatio = TURN_GEAR_RATIO;
    turnConfig.Feedback.SensorToMechanismRatio = 1;
    turnConfig.Feedback.FeedbackRotorOffset =
        0.0; // Is this right? I think CANcoder config handles this

    turnConfig.Slot0.kV = 2.5678;
    turnConfig.Slot0.kA = 0.0;
    turnConfig.Slot0.kS = 0.16677;
    turnConfig.Slot0.kP = 75;
    turnConfig.Slot0.kD = 0;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;

    turnTalon.getConfigurator().apply(turnConfig);
    setTurnBrakeMode(true);

    // Status Signals
    timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

    drivePosition = driveTalon.getPosition();
    drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(driveTalon, driveTalon.getPosition());
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getStatorCurrent();

    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnPosition = turnTalon.getPosition();
    turnPositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(turnTalon, turnTalon.getPosition());
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnCurrent = turnTalon.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        Module.ODOMETRY_FREQUENCY, drivePosition, turnPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, // Bus is CAN FD, so consider increasing this
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);
    driveTalon.optimizeBusUtilization();
    turnTalon.optimizeBusUtilization();
    cancoder.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnPosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);

    inputs.drivePositionMeters = drivePosition.getValueAsDouble() / DRIVE_GEAR_RATIO;
    inputs.driveVelocityMetersPerSec = driveVelocity.getValueAsDouble() / DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = new double[] {driveCurrent.getValueAsDouble()};

    inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = new double[] {turnCurrent.getValueAsDouble()};

    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).limit(100).toArray();
    inputs.odometryDrivePositionsMeters =
        drivePositionQueue.stream().mapToDouble(Double::doubleValue).limit(100).toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map(Rotation2d::fromRotations)
            .limit(100)
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveTalon.setControl(driveVoltage.withOutput(volts));
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnTalon.setControl(turnVoltage.withOutput(volts));
  }

  @Override
  public void setDriveSetpoint(double metersPerSecond) {
    driveTalon.setControl(drivePIDF.withVelocity(metersPerSecond));
  }

  @Override
  public void setTurnSetpoint(Rotation2d rotation) {
    turnTalon.setControl(turnPID.withPosition(rotation.getRotations()));
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted =
        isDriveMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    driveTalon.getConfigurator().apply(config);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted =
        isTurnMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    turnTalon.getConfigurator().apply(config);
  }
}
