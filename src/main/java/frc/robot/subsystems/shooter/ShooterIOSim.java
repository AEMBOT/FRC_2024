package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {

  private final DCMotorSim motorSim = new DCMotorSim(DCMotor.getNEO(4), 1, 0.01);
  private final PIDController shooterPID = new PIDController(1.0, 0.0, 0.0);

  private double shooterAppliedVolts;
  private boolean openLoop = false;
  private double ffVolts = 0.0;

  public void updateInputs(ShooterIOInputs inputs) {

    if (openLoop) {
      shooterAppliedVolts =
          MathUtil.clamp(
              shooterPID.calculate(motorSim.getAngularVelocityRPM()) + ffVolts, -12.0, 12.0);
      motorSim.setInputVoltage(shooterAppliedVolts);
    }

    motorSim.update(0.02);

    inputs.shooterVelocityRPM = new double[] {motorSim.getAngularVelocityRPM()};
    inputs.shooterAppliedVolts = new double[] {shooterAppliedVolts};
    inputs.shooterCurrentAmps = new double[] {motorSim.getCurrentDrawAmps()};
    inputs.openLoopStatus = openLoop;
  }

  public void setVoltage(double volts) {
    openLoop = true;
    shooterAppliedVolts = volts;
    motorSim.setInputVoltage(volts);
  }

  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    openLoop = false;
    shooterPID.setSetpoint(velocityRadPerSec);
    this.ffVolts = ffVolts;
  }

  public void stop() {
    setVoltage(0.0);
  }
}
