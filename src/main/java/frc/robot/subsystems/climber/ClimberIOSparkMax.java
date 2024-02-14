package frc.robot.subsystems.climber;

import static frc.robot.Constants.ClimberConstants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import frc.robot.subsystems.climber.ClimberIO.ClimberIOInputs;
import org.littletonrobotics.junction.Logger;

public class ClimberIOSparkMax implements ClimberIO {
  private static final double GEAR_RATIO = 1.5;
  private double appliedVolts = 0;
  private boolean openLoop = false;

  // choosing right as the main motor and left as the follower motor

  private final CANSparkMax m_winchMotorRight = new CANSparkMax(winchMotorRightCanID, MotorType.kBrushless);
  private final CANSparkMax m_winchMotorLeft = new CANSparkMax(winchMotorLeftCanID, MotorType.kBrushless);
  private final RelativeEncoder encoderRight = m_winchMotorRight.getEncoder();
  private final RelativeEncoder encoderLeft = m_winchMotorLeft.getEncoder();

  private final ElevatorFeedforward climberFFModelUp =
      new ElevatorFeedforward(0.0, 0.0379, 5.85, 0.04); // TODO: tune
  private final ElevatorFeedforward climberFFModelDown =
      new ElevatorFeedforward(0, 0, 0); // TODO: tune
  private final PIDController pidControllerUp = new PIDController(1, 0, 0); // TODO: tune
  private final PIDController pidControllerDown = new PIDController(1, 0, 0); // TODO: tune

  private final ExponentialProfile climberProfile =
      new ExponentialProfile(
          ExponentialProfile.Constraints.fromCharacteristics(
              10, climberFFModelUp.kv, climberFFModelUp.ka));
  private ExponentialProfile.State climberGoal;
  private ExponentialProfile.State climberSetpoint;


  public ClimberIOSparkMax() {
    m_winchMotorRight.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_winchMotorLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);

    m_winchMotorRight.setSmartCurrentLimit(homingCurrentLimit);
    m_winchMotorLeft.setSmartCurrentLimit(
        homingCurrentLimit); // logic for homing vs extended is built in now

    m_winchMotorRight.setInverted(false);
    m_winchMotorLeft.setInverted(false);

    m_winchMotorRight.setCANTimeout(250);
    m_winchMotorLeft.setCANTimeout(250);

    climberGoal = new ExponentialProfile.State(encoderRight.getPosition(), 0);
    climberSetpoint = new ExponentialProfile.State(encoderRight.getPosition(), 0);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    double currentVelocity = climberSetpoint.velocity;
    // change 0.02 for constants.update period when merged properly in climbersetpoint definitionand
    // feedforwardup definition
    climberSetpoint = climberProfile.calculate(0.02, climberSetpoint, climberGoal);
    double feedForwardUp =
        climberFFModelUp.calculate(currentVelocity, climberSetpoint.velocity, 0.02);
    double feedForwardDown =
        climberFFModelDown.calculate(currentVelocity, climberSetpoint.velocity, 0.02);
    double pidOutputUp =
        pidControllerUp.calculate(encoderRight.getPosition(), climberSetpoint.position);
    double pidOutputDown =
        pidControllerDown.calculate(encoderRight.getPosition(), climberSetpoint.position);

    Logger.recordOutput("Climber/CalculatedFFVoltsUp", feedForwardUp);
    Logger.recordOutput("Climber/CalculatedFFVoltsDown", feedForwardDown);
    Logger.recordOutput("Climber/PIDCommandVoltsUp", pidOutputUp);
    Logger.recordOutput("Climber/PIDCommandVoltsDown", pidOutputDown);
    if(!openLoop) {
      if (encoderRight.getPosition() > climberSetpoint.position) {
        appliedVolts = feedForwardDown + pidOutputDown;
      } else {
        appliedVolts = feedForwardUp + pidOutputUp;
      }
    }
    inputs.climberAbsoluteVelocityMetersPerSec = encoderRight.getVelocity();
    inputs.climberAppliedVolts = appliedVolts;
    inputs.climberCurrentAmps =
        new double[] {m_winchMotorRight.getOutputCurrent(), m_winchMotorLeft.getOutputCurrent()};
    inputs.climberGoalPosition = climberGoal.position;
    inputs.climberSetpointPosition = climberSetpoint.position;
    inputs.climberSetpointVelocity = climberSetpoint.velocity;
  }

  @Override
  public void setHoming(boolean homingBool) {
    if (homingBool) {
      m_winchMotorRight.setSmartCurrentLimit(homingCurrentLimit);
      m_winchMotorLeft.setSmartCurrentLimit(homingCurrentLimit);
      setVoltage(-2);
    } else {
      m_winchMotorRight.setSmartCurrentLimit(extendCurrentLimit);
      m_winchMotorLeft.setSmartCurrentLimit(extendCurrentLimit);
    }
  }

  @Override
  public void setPosition(double position) {
    climberGoal = new ExponentialProfile.State(position, 0);
    openLoop = false;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
    openLoop = true;
    m_winchMotorRight.setVoltage(appliedVolts);
    m_winchMotorLeft.setVoltage(appliedVolts);
  }

  @Override
  public void setVelocity(double velocityMetersPerSec) {
    openLoop = true;
    m_winchMotorLeft.set(velocityMetersPerSec);
    m_winchMotorRight.set(velocityMetersPerSec);
  }

  @Override
  public void stop() {
    setVoltage(0);
  }

  @Override
  public void resetEncoder(double position) {
    encoderRight.setPosition(0);
    encoderLeft.setPosition(0);
  }
}
