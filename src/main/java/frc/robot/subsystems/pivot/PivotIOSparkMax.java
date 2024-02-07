package frc.robot.subsystems.pivot;

import static frc.robot.Constants.PivotConstants.*;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class PivotIOSparkMax implements PivotIO {
  private boolean openLoop = false;
  private double appliedVolts = 0.0;
  private final CANSparkMax leftMotor =
      new CANSparkMax(LEFT_MOTOR_PORT, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax rightMotor =
      new CANSparkMax(RIGHT_MOTOR_PORT, CANSparkLowLevel.MotorType.kBrushless);
  private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(ENCODER_PORT);
  private final PIDController controller = new PIDController(50, 0.0, 0.0);
  private final ArmFeedforward pivotFFModel = new ArmFeedforward(0.0, 0.0379, 5.85, 0.04);
  private final ExponentialProfile pivotProfile =
      new ExponentialProfile(
          ExponentialProfile.Constraints.fromCharacteristics(10, pivotFFModel.kv, pivotFFModel.ka));
  private ExponentialProfile.State pivotGoal;
  private ExponentialProfile.State pivotSetpoint;

  public PivotIOSparkMax() {
    pivotGoal = new ExponentialProfile.State(1.05, 0);
    pivotSetpoint = new ExponentialProfile.State(1.05, 0);
  }

  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(PivotIOInputs inputs) {
    if (!openLoop) {
      double currentVelocity = pivotSetpoint.velocity;
      pivotSetpoint = pivotProfile.calculate(UPDATE_PERIOD, pivotSetpoint, pivotGoal);
      double feedForward =
          pivotFFModel.calculate(
              pivotSetpoint.position,
              pivotSetpoint.velocity,
              (pivotSetpoint.velocity - currentVelocity) / UPDATE_PERIOD);
      appliedVolts =
          MathUtil.clamp(
              controller.calculate(absoluteEncoder.getAbsolutePosition(), pivotSetpoint.position)
                  + feedForward,
              -VOLT_CLAMP,
              VOLT_CLAMP);
    }
    leftMotor.setVoltage(appliedVolts);
    leftMotor.setVoltage(appliedVolts);
    inputs.pivotAbsolutePositionRad =
        (absoluteEncoder.getAbsolutePosition() - absoluteEncoder.getPositionOffset())
            / absoluteEncoder.getDistancePerRotation();
    inputs.pivotAppliedVolts = appliedVolts;
    inputs.pivotCurrentAmps =
        new double[] {leftMotor.getOutputCurrent(), rightMotor.getOutputCurrent()};
    inputs.pivotAbsoluteVelocityRadPerSec = leftMotor.getEncoder().getVelocity();
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
}
