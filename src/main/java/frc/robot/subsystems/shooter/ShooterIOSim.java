package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO{

    private final DCMotorSim motorSim = new DCMotorSim(DCMotor.getNEO(4), 1, 0.01);
    private PIDController shooterPID = new PIDController(0.0, 0.0, 0.0);

    private double shooterAppliedVolts;
    private boolean closedLoop = false;
    private double ffVolts = 0.0;

    public void updateInputs(ShooterIOInputs inputs) {

        if(closedLoop){
            shooterAppliedVolts = MathUtil.clamp(shooterPID.calculate(motorSim.getAngularVelocityRadPerSec()) + ffVolts, -12.0, 12.0);
            motorSim.setInputVoltage(shooterAppliedVolts);
        }

        motorSim.update(0.02);
        
        inputs.shooterVelocityRadPerSec = motorSim.getAngularVelocityRadPerSec();
        inputs.shooterAppliedVolts = shooterAppliedVolts;
        inputs.shooterCurrentAmps = new double[] {motorSim.getCurrentDrawAmps()};
    }

    public void setVoltage(double volts) {
        closedLoop = false;
        shooterAppliedVolts = volts;
        motorSim.setInputVoltage(volts);
    }

    public void setVelocity(double velocityRadPerSec, double ffVolts) {
        closedLoop = true;
        shooterPID.setSetpoint(velocityRadPerSec);
        this.ffVolts = ffVolts;
    }

    public void stop() {
        setVoltage(0.0);
    }

    public void configurePID(double kP, double kI, double kD) {
        shooterPID.setPID(kP, kI, kD);
    }
}
