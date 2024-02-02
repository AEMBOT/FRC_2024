package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

public class ShooterIOReal implements ShooterIO  {
  private boolean closedLoop = false;
  private double voltage = 0.0;
  private double targetVelocity = 0.0;
  private double feedForward = 0.0;
  private PIDController controller = new PIDController(0.0, 0.0, 0.0);
  private CANSparkMax leftMotor1 = new CANSparkMax(0, CANSparkMax.MotorType.kBrushless);
  private CANSparkMax leftMotor2 = new CANSparkMax(0, CANSparkMax.MotorType.kBrushless);
  private CANSparkMax rightMotor1 = new CANSparkMax(0, CANSparkMax.MotorType.kBrushless);
  private CANSparkMax rightMotor2 = new CANSparkMax(0, CANSparkMax.MotorType.kBrushless);
  /** Updates the set of loggable inputs. */
  public void updateInputs(ShooterIOInputs inputs) {
    if(closedLoop){
      voltage = MathUtil.clamp(controller.calculate(leftMotor1.getEncoder().getVelocity(), targetVelocity) + feedForward, -10, 10);
    }
    leftMotor1.setVoltage(voltage);
    leftMotor2.setVoltage(voltage);
    rightMotor1.setVoltage(voltage);
    rightMotor2.setVoltage(voltage);
    inputs.shooterAppliedVolts = voltage;
    inputs.shooterCurrentAmps = new double[]{
            leftMotor1.getOutputCurrent(),
            leftMotor2.getOutputCurrent(),
            rightMotor1.getOutputCurrent(),
            rightMotor2.getOutputCurrent()
    };
    inputs.shooterVelocityRadPerSec = leftMotor1.getEncoder().getVelocity();
  }

  /** Run open loop at the specified voltage. Primarily for characterization. */
  public void setVoltage(double volts) {
    closedLoop = false;
    voltage = volts;

  }

  /** Run closed loop at the specified velocity. */
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    closedLoop = true;
    targetVelocity = velocityRadPerSec;
    feedForward = ffVolts;
  }

  /** Stop in open loop. */
  public void stop() {
    setVoltage(0.0);
  }

  /** Set velocity PID constants. */
  public void configurePID(double kP, double kI, double kD) {
    controller.setPID(kP, kI, kD);
  }
}
