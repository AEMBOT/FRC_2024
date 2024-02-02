package frc.robot.subsystems.pivot;

import static com.revrobotics.CANSparkBase.ControlType.kVelocity;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;

public class PivotIOReal implements PivotIO {

  private boolean openLoop = false;
  private double appliedVolts = 0.0;

  private final CANSparkMax motorLeader = new CANSparkMax(69, MotorType.kBrushless);
  private final CANSparkMax motorFollower = new CANSparkMax(420, MotorType.kBrushless);
  private final DutyCycleEncoder encoder = new DutyCycleEncoder(1);
  private final Encoder velEncoder = new Encoder(2, 3);
  private final ArmFeedforward pivotFFModel = new ArmFeedforward(0.0, 0.0379, 5.85, 0.04);
  private final ExponentialProfile pivotProfile =
      new ExponentialProfile(
          ExponentialProfile.Constraints.fromCharacteristics(10, pivotFFModel.kv, pivotFFModel.ka));
  private ExponentialProfile.State pivotGoal;
  private ExponentialProfile.State pivotSetpoint;
  private SparkPIDController topMotorPID;

  public PivotIOReal() {
    pivotGoal = new ExponentialProfile.State(1.05, 0);
    pivotSetpoint = new ExponentialProfile.State(1.05, 0);
    motorLeader.setSmartCurrentLimit(60);
    motorFollower.setSmartCurrentLimit(60);
    topMotorPID = motorLeader.getPIDController();
    topMotorPID.setP(1); // TODO tune
    motorFollower.follow(motorLeader, true);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    if (!openLoop) {
      double currentVelocity = pivotSetpoint.velocity;
      pivotSetpoint = pivotProfile.calculate(0.02, pivotSetpoint, pivotGoal);
      double feedForward =
          pivotFFModel.calculate(
              pivotSetpoint.position,
              pivotSetpoint.velocity,
              (pivotSetpoint.velocity - currentVelocity) / 0.02);

      topMotorPID.setReference(
          pivotSetpoint.velocity, kVelocity, 0, feedForward, ArbFFUnits.kVoltage);
    }
    inputs.pivotAbsolutePositionRad = encoder.get();
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
    motorLeader.setVoltage(volts);
  }

  /** Stop in open loop. */
  @Override
  public void stop() {
    openLoop = true;
    setVoltage(0.0);
  }
}
