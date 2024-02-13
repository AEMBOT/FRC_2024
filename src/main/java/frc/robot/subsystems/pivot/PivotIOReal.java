package frc.robot.subsystems.pivot;

import static edu.wpi.first.math.MathUtil.clamp;
import static edu.wpi.first.wpilibj.Timer.delay;
import static frc.robot.Constants.PivotConstants.PIVOT_MAX_POS_RAD;
import static frc.robot.Constants.PivotConstants.PIVOT_MIN_POS_RAD;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class PivotIOReal implements PivotIO {

  private boolean openLoop = true;
  private double appliedVolts = 0.0;

  private final CANSparkMax motorLeader = new CANSparkMax(11, MotorType.kBrushless);
  private final CANSparkMax motorFollower = new CANSparkMax(10, MotorType.kBrushless);
  private final DutyCycleEncoder encoder = new DutyCycleEncoder(3);
  private final Encoder velEncoder = new Encoder(2, 1, true);
  private final ArmFeedforward pivotFFModel =
      new ArmFeedforward(0.0, 0.0379, 5.85, 0.04); // TODO tune
  private final PIDController pidController = new PIDController(1, 0, 0); // TODO tune
  private final ExponentialProfile pivotProfile =
      new ExponentialProfile(
          ExponentialProfile.Constraints.fromCharacteristics(10, pivotFFModel.kv, pivotFFModel.ka));
  private ExponentialProfile.State pivotGoal;
  private ExponentialProfile.State pivotSetpoint;

  public PivotIOReal() {
    pivotGoal = new ExponentialProfile.State(getAbsoluteEncoderPosition(), 0);
    pivotSetpoint = new ExponentialProfile.State(getAbsoluteEncoderPosition(), 0);

    motorLeader.restoreFactoryDefaults();
    motorFollower.restoreFactoryDefaults();

    delay(0.25);

    motorLeader.setIdleMode(CANSparkBase.IdleMode.kBrake);
    motorFollower.setIdleMode(CANSparkBase.IdleMode.kBrake);

    // I have no clue what's going on with invert settings, but both inverted and invert follow
    // works
    motorLeader.setInverted(true);
    motorFollower.setInverted(true);

    motorLeader.setSmartCurrentLimit(60);
    motorFollower.setSmartCurrentLimit(60);

    delay(0.25);

    motorFollower.follow(motorLeader, true);

    encoder.setPositionOffset(
        2.85765 / (2 * Math.PI)); // Convert from offset rads to offset rotations
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    if (!openLoop) {
      double currentVelocity = pivotSetpoint.velocity;
      pivotSetpoint = pivotProfile.calculate(Constants.UPDATE_PERIOD, pivotSetpoint, pivotGoal);
      double feedForward =
          pivotFFModel.calculate(
              pivotSetpoint.position,
              pivotSetpoint.velocity,
              (pivotSetpoint.velocity - currentVelocity) / Constants.UPDATE_PERIOD);
      double pidOutput =
          pidController.calculate(getAbsoluteEncoderPosition(), pivotSetpoint.position);

      Logger.recordOutput("Pivot/CalculatedFFVolts", feedForward);
      Logger.recordOutput("Pivot/PIDCommandVolts", pidOutput);

      appliedVolts = feedForward + pidOutput;
    }

    if (getAbsoluteEncoderPosition() < PIVOT_MIN_POS_RAD) {
      appliedVolts = clamp(appliedVolts, 0, Double.MAX_VALUE);
    }
    if (getAbsoluteEncoderPosition() > PIVOT_MAX_POS_RAD) {
      appliedVolts = clamp(appliedVolts, Double.MIN_VALUE, 0);
    }

    motorLeader.setVoltage(appliedVolts);

    inputs.pivotAbsolutePositionRad = getAbsoluteEncoderPosition();
    inputs.pivotAppliedVolts = appliedVolts;
    inputs.pivotCurrentAmps =
        new double[] {motorLeader.getOutputCurrent(), motorFollower.getOutputCurrent()};
    inputs.pivotAbsoluteVelocityRadPerSec = velEncoder.getRate();
    inputs.pivotGoalPosition = pivotGoal.position;
    inputs.pivotSetpointPosition = pivotSetpoint.position;
    inputs.pivotSetpointVelocity = pivotSetpoint.velocity;
    inputs.openLoopStatus = openLoop;
  }

  /** Sets the angle of the pivot, in radians. */
  @Override
  public void setPosition(double positionRad) {
    openLoop = false;
    pivotGoal = new ExponentialProfile.State(positionRad, 0);
  }

  /** Run open loop at the specified voltage. */
  @Override
  public void setVoltage(double volts) {
    openLoop = true;
    appliedVolts = volts;
  }

  /** Stop in open loop. */
  @Override
  public void stop() {
    openLoop = true;
    setVoltage(0.0);
  }

  private double getAbsoluteEncoderPosition() {
    return Units.rotationsToRadians(encoder.getAbsolutePosition() - encoder.getPositionOffset());
  }
}
