package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class PivotIOSim implements PivotIO {
  private boolean openLoop = false;
  private double feedForward = 0.0;
  private double appliedVolts = 0.0;
  private DCMotorSim sim = new DCMotorSim(DCMotor.getNEO(2), 1, 1); // Numbers TODO
  private PIDController controller = new PIDController(0.0,0.0,0.0);
  private final ArmFeedforward pivotFFModel = new ArmFeedforward(0.0, 0.0, 0.0, 0.0);
  private ExponentialProfile pivotProfile =
          new ExponentialProfile(
                  ExponentialProfile.Constraints.fromCharacteristics(10, 0.0, 0.0));
  private ExponentialProfile.State pivotGoal = new ExponentialProfile.State();
  private ExponentialProfile.State pivotSetpoint = new ExponentialProfile.State();


  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(PivotIOInputs inputs) {
    if (!openLoop){
      double currentVelocity = pivotSetpoint.velocity;
      pivotSetpoint = pivotProfile.calculate(0.02,pivotSetpoint,pivotGoal);
      feedForward = pivotFFModel.calculate(pivotSetpoint.position, pivotSetpoint.velocity, (pivotSetpoint.velocity - currentVelocity) / 0.02);
      appliedVolts = MathUtil.clamp(controller.calculate(pivotSetpoint.position) + feedForward, -12.0, 12.0);
    }
    sim.setInputVoltage(appliedVolts);
    sim.update(0.2);
    inputs.pivotAbsolutePositionRad = sim.getAngularPositionRad();
    inputs.pivotAppliedVolts = appliedVolts;
    inputs.pivotCurrentAmps = new double[]{sim.getCurrentDrawAmps(), sim.getCurrentDrawAmps()};
    inputs.pivotAbsoluteVelocityRadPerSec = sim.getAngularVelocityRadPerSec();
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
    setVoltage(0.0);
  }

  /** Set position PID constants. */
  @Override
  public void configurePID(double kP, double kI, double kD) {
    controller.setPID(kP, kI, kD);
  }
  @Override
  public void resetExp(double kV, double kA) {
    pivotProfile = new ExponentialProfile(ExponentialProfile.Constraints.fromCharacteristics(10, kV, kA));
  }
}
