package frc.robot.subsystems.shooter;

import static com.revrobotics.CANSparkBase.ControlType.kVelocity;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;

public class ShooterIOReal implements ShooterIO {
  private boolean closedLoop = false;
  private double voltage = 0.0;
  private final CANSparkMax topMotorLeader = new CANSparkMax(0, CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax topMotorFollower = new CANSparkMax(0, CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax bottomMotorLeader =
      new CANSparkMax(0, CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax bottomMotorFollower =
      new CANSparkMax(0, CANSparkMax.MotorType.kBrushless);

  private SparkPIDController topMotorPID;
  private SparkPIDController bottomMotorPID;

  public ShooterIOReal() {
    // Tune acceptable current limit, don't want to use all power if shoot while moving
    topMotorLeader.setSmartCurrentLimit(60);
    topMotorFollower.setSmartCurrentLimit(60);
    bottomMotorLeader.setSmartCurrentLimit(60);
    bottomMotorFollower.setSmartCurrentLimit(60);

    topMotorLeader.getEncoder().setVelocityConversionFactor(2);
    topMotorFollower.getEncoder().setVelocityConversionFactor(2);
    bottomMotorLeader.getEncoder().setVelocityConversionFactor(2);
    bottomMotorFollower.getEncoder().setVelocityConversionFactor(2);

    topMotorPID = topMotorLeader.getPIDController();
    bottomMotorPID = bottomMotorLeader.getPIDController();

    topMotorPID.setP(1); // TODO tune
    bottomMotorPID.setP(1);

    topMotorFollower.follow(topMotorLeader, true);
    bottomMotorFollower.follow(bottomMotorLeader, true);
  }

  /** Updates the set of loggable inputs. */
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.shooterAppliedVolts =
        new double[] {
          topMotorLeader.getAppliedOutput() * topMotorLeader.getBusVoltage(),
          bottomMotorLeader.getAppliedOutput() * bottomMotorLeader.getBusVoltage()
        };
    inputs.shooterCurrentAmps =
        new double[] {
          topMotorLeader.getOutputCurrent(),
          topMotorFollower.getOutputCurrent(),
          bottomMotorLeader.getOutputCurrent(),
          bottomMotorFollower.getOutputCurrent()
        };
    inputs.shooterVelocityRadPerSec =
        new double[] {
          topMotorLeader.getEncoder().getVelocity(), bottomMotorLeader.getEncoder().getVelocity()
        };
  }

  /** Run open loop at the specified voltage. Primarily for characterization. */
  public void setVoltage(double volts) {
    closedLoop = false;
    topMotorLeader.setVoltage(volts);
    bottomMotorLeader.setVoltage(volts);
  }

  /** Run closed loop at the specified velocity. */
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    closedLoop = true;
    topMotorPID.setReference(velocityRadPerSec, kVelocity, 0, ffVolts, ArbFFUnits.kVoltage);
    bottomMotorPID.setReference(velocityRadPerSec, kVelocity, 0, ffVolts, ArbFFUnits.kVoltage);
  }

  /** Stop in open loop. */
  public void stop() {
    setVoltage(0.0);
  }
}
