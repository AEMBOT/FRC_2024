package frc.robot.subsystems.climber;

import static edu.wpi.first.wpilibj.Timer.delay;
import static frc.robot.Constants.ClimberConstants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.climber.ClimberIO.ClimberIOInputs;
import org.littletonrobotics.junction.Logger;

public class ClimberIOSparkMax implements ClimberIO {
  private static final double GEAR_RATIO = 15;
  private static final double SPOOL_DIAMETER = Units.inchesToMeters(1.0);
  private double climberSetpoint = 0;
  private boolean openLoop = false;

  private final LinearFilter currentFilterLeft = LinearFilter.singlePoleIIR(0.2, 0.02);
  private final LinearFilter currentFilterRight = LinearFilter.singlePoleIIR(0.2, 0.02);
  private double currentFilterLeftValue = 0.0;
  private double currentFilterRightValue = 0.0;

  private final CANSparkMax m_winchMotorRight =
      new CANSparkMax(winchMotorRightCanID, MotorType.kBrushless);
  private final CANSparkMax m_winchMotorLeft =
      new CANSparkMax(winchMotorLeftCanID, MotorType.kBrushless);
  private final RelativeEncoder encoderRight = m_winchMotorRight.getEncoder();
  private final RelativeEncoder encoderLeft = m_winchMotorLeft.getEncoder();
  private final PIDController pidControllerUp = new PIDController(100, 0, 0); // TODO: tune
  private final PIDController pidControllerDown = new PIDController(200, 0, 0); // TODO: tune

  public ClimberIOSparkMax() {
    m_winchMotorLeft.restoreFactoryDefaults();
    m_winchMotorRight.restoreFactoryDefaults();

    delay(0.25);

    m_winchMotorRight.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_winchMotorLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);

    m_winchMotorRight.setSmartCurrentLimit(homingCurrentLimit);
    m_winchMotorLeft.setSmartCurrentLimit(homingCurrentLimit);

    m_winchMotorRight.setInverted(true);
    m_winchMotorLeft.setInverted(false);

    encoderLeft.setPositionConversionFactor(Math.PI * SPOOL_DIAMETER / GEAR_RATIO); // Rot to m
    encoderRight.setPositionConversionFactor(Math.PI * SPOOL_DIAMETER / GEAR_RATIO); // Rot to m
    encoderLeft.setVelocityConversionFactor(
        Math.PI * SPOOL_DIAMETER / GEAR_RATIO / 60); // RPM to m/s
    encoderRight.setVelocityConversionFactor(
        Math.PI * SPOOL_DIAMETER / GEAR_RATIO / 60); // RPM to m/s

    m_winchMotorRight.setCANTimeout(250);
    m_winchMotorLeft.setCANTimeout(250);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.climberLeftAppliedVolts =
        m_winchMotorLeft.getAppliedOutput() * m_winchMotorLeft.getBusVoltage();
    inputs.climberRightAppliedVolts =
        m_winchMotorRight.getAppliedOutput() * m_winchMotorRight.getBusVoltage();
    inputs.climberLeftVelocityMetersPerSec = encoderLeft.getVelocity();
    inputs.climberRightVelocityMetersPerSec = encoderRight.getVelocity();
    inputs.climberLeftPositionMeters = encoderLeft.getPosition();
    inputs.climberRightPositionMeters = encoderRight.getPosition();
    inputs.climberCurrentAmps =
        new double[] {m_winchMotorLeft.getOutputCurrent(), m_winchMotorRight.getOutputCurrent()};
    inputs.climberSetpointPosition = climberSetpoint;
    inputs.openLoopStatus = openLoop;

    currentFilterLeftValue = currentFilterLeft.calculate(m_winchMotorLeft.getOutputCurrent());
    currentFilterRightValue = currentFilterRight.calculate(m_winchMotorRight.getOutputCurrent());

    Logger.recordOutput("Current Filter Left Calculated Value", currentFilterLeftValue);
    Logger.recordOutput("Current Filter Right Calculated Value", currentFilterRightValue);
  }

  @Override
  public void setHoming(boolean homingBool) {
    if (homingBool) {
      m_winchMotorRight.setSmartCurrentLimit(homingCurrentLimit);
      m_winchMotorLeft.setSmartCurrentLimit(homingCurrentLimit);
    } else {
      m_winchMotorRight.setSmartCurrentLimit(extendCurrentLimit);
      m_winchMotorLeft.setSmartCurrentLimit(extendCurrentLimit);
    }
  }

  @Override
  public void setPosition(double position) {
    climberSetpoint = position;
    m_winchMotorLeft.setVoltage(
        encoderLeft.getPosition() > climberSetpoint
            ? pidControllerDown.calculate(encoderLeft.getPosition(), position)
            : pidControllerUp.calculate(encoderLeft.getPosition(), position));

    m_winchMotorRight.setVoltage(
        encoderRight.getPosition() > climberSetpoint
            ? pidControllerDown.calculate(encoderRight.getPosition(), position)
            : pidControllerUp.calculate(encoderRight.getPosition(), position));

    openLoop = false;
  }

  @Override
  public void setPositionClimbing(double position) {
    if (currentFilterLeftValue > 50) {
      climberSetpoint = position;

      m_winchMotorLeft.setVoltage(
          encoderLeft.getPosition() > climberSetpoint
              ? Math.min(pidControllerDown.calculate(encoderLeft.getPosition(), position), 2)
              : Math.min(pidControllerUp.calculate(encoderLeft.getPosition(), position), 2));

      m_winchMotorRight.setVoltage(
          encoderLeft.getPosition() > climberSetpoint
              ? pidControllerDown.calculate(encoderLeft.getPosition(), position)
              : pidControllerUp.calculate(encoderLeft.getPosition(), position));
    } else if (currentFilterRightValue > 50) {
      climberSetpoint = position;

      m_winchMotorLeft.setVoltage(
          encoderLeft.getPosition() > climberSetpoint
              ? pidControllerDown.calculate(encoderLeft.getPosition(), position)
              : pidControllerUp.calculate(encoderLeft.getPosition(), position));

      m_winchMotorRight.setVoltage(
          encoderRight.getPosition() > climberSetpoint
              ? Math.min(pidControllerDown.calculate(encoderRight.getPosition(), position), 2)
              : Math.min(pidControllerUp.calculate(encoderRight.getPosition(), position), 2));
    } else {
      setPosition(position);
    }
    openLoop = false;
  }

  public double currentLimitedClimbing() {
    // 1 if left climber is current limited
    // 2 if right climber is current limited
    // 3 if both
    if (currentFilterLeftValue > 50 && currentFilterRightValue > 50) {
      return 3;
    } else if (currentFilterLeftValue > 50) {
      return 1;
    } else if (currentFilterRightValue > 50) {
      return 2;
    }
    return 0;
  }

  @Override
  public void setVoltage(double volts) {
    openLoop = true;
    m_winchMotorRight.setVoltage(volts);
    m_winchMotorLeft.setVoltage(volts);
  }

  @Override
  public void resetEncoder(final double position) {
    encoderRight.setPosition(position);
    encoderLeft.setPosition(position);
  }

  public boolean isCurrentLimited() {
    return currentFilterLeftValue > 20 && currentFilterRightValue > 20;
  }
}
