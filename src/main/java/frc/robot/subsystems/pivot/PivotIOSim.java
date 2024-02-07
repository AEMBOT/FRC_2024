package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class PivotIOSim implements PivotIO {
  private boolean openLoop = false;
  private double appliedVolts = 0.0;
  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(DCMotor.getNEO(2), 300, 0.17, 0.508, 0.0, Math.PI, true, 1.05);

  private final PIDController controller = new PIDController(50, 0.0, 0.0);
  private final ArmFeedforward pivotFFModel = new ArmFeedforward(0.0, 0.0379, 5.85, 0.04);
  private final ExponentialProfile pivotProfile =
      new ExponentialProfile(
          ExponentialProfile.Constraints.fromCharacteristics(10, pivotFFModel.kv, pivotFFModel.ka));
  private ExponentialProfile.State pivotGoal;
  private ExponentialProfile.State pivotSetpoint;

  public PivotIOSim() {
    pivotGoal = new ExponentialProfile.State(1.05, 0);
    pivotSetpoint = new ExponentialProfile.State(1.05, 0);
  }

  /** Updates the set of loggable inputs. */
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
      appliedVolts =
          MathUtil.clamp(
              controller.calculate(sim.getAngleRads(), pivotSetpoint.position) + feedForward,
              -12.0,
              12.0);
    }
    sim.setInputVoltage(appliedVolts);
    sim.update(0.02);
    inputs.pivotAbsolutePositionRad = sim.getAngleRads();
    inputs.pivotAppliedVolts = appliedVolts;
    inputs.pivotCurrentAmps = new double[] {sim.getCurrentDrawAmps()};
    inputs.pivotAbsoluteVelocityRadPerSec = sim.getVelocityRadPerSec();
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
