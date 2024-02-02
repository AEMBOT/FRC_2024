package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.climber.ClimberIO.ClimberIOInputs;
import frc.robot.Constants;
import static frc.robot.Constants.ClimberConstants.*;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class ClimberIOSparkMax implements ClimberIO {
  private static final double GEAR_RATIO = 1.5;

  private final CANSparkMax m_winchMotorRight = new CANSparkMax(10, MotorType.kBrushless);
  private final CANSparkMax m_winchMotorLeft = new CANSparkMax(11, MotorType.kBrushless);
  private final RelativeEncoder encoder = m_winchMotorRight.getEncoder();
  private final SparkPIDController pid = m_winchMotorRight.getPIDController();

  public ClimberIOSparkMax() {
     m_winchMotorRight.restoreFactoryDefaults();
    m_winchMotorLeft.restoreFactoryDefaults();
    
    m_winchMotorRight.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_winchMotorLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);

    m_winchMotorRight.setSmartCurrentLimit(extendCurrentLimit);
    m_winchMotorLeft.setSmartCurrentLimit(extendCurrentLimit); //make sure to have logic for homing current limit

    m_winchMotorRight.setInverted(false);
    m_winchMotorLeft.setInverted(false);

    m_winchMotorRight.setSmartCurrentLimit(extendCurrentLimit);
    m_winchMotorLeft.setSmartCurrentLimit(extendCurrentLimit);

    m_winchMotorRight.burnFlash();
    m_winchMotorLeft.burnFlash();

    m_winchMotorRight.setCANTimeout(250);
    m_winchMotorLeft.setCANTimeout(250);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.climberPositionRad = Units.rotationsToRadians(encoder.getPosition() / GEAR_RATIO);
    inputs.climberAbsoluteVelocityRadPerSec=
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / GEAR_RATIO);
    inputs.climberAppliedVolts = m_winchMotorRight.getAppliedOutput() * m_winchMotorRight.getBusVoltage();
    inputs.climberCurrentAmps = new double[] {m_winchMotorRight.getOutputCurrent(), m_winchMotorLeft.getOutputCurrent()};
  }

  @Override
  public void setVoltage(double volts) {
    m_winchMotorRight.setVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    pid.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * GEAR_RATIO,
        ControlType.kVelocity,
        0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    m_winchMotorRight.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setP(kP, 0);
    pid.setI(kI, 0);
    pid.setD(kD, 0);
    pid.setFF(0, 0);
  }
}