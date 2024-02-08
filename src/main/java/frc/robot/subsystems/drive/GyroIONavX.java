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

import static frc.robot.subsystems.drive.Module.ODOMETRY_FREQUENCY;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import java.util.Queue;

/** IO implementation for NavX */
public class GyroIONavX implements GyroIO {
  private final AHRS navX = new AHRS(SPI.Port.kMXP, (byte) ODOMETRY_FREQUENCY); // 200Hz update rate
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  public GyroIONavX() {
    navX.reset();
    yawPositionQueue = SparkMaxOdometryThread.getInstance().registerSignal(() -> navX.getAngle());
    yawTimestampQueue =
        SparkMaxOdometryThread.getInstance().registerSignal(() -> navX.getLastSensorTimestamp());
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = navX.isConnected();
    inputs.yawPosition = Rotation2d.fromDegrees(navX.getAngle());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(navX.getRate());

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
