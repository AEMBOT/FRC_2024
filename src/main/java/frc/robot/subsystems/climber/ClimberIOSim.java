// change between sim and sparkmax files

package frc.robot.subsystems.climber;

import static frc.robot.Constants.ClimberConstants.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ClimberIOSim implements ClimberIO {
  private double appliedVolts = 0;

  // choosing right as the main motor and left as the follower motor
  private final DCMotor m_elevatorGearbox = DCMotor.getNEO(2);

  private final ElevatorSim sim =
      new ElevatorSim(
          m_elevatorGearbox,
          15,
          5,
          Units.inchesToMeters(0.5),
          Units.inchesToMeters(8),
          Units.inchesToMeters(24),
          true,
          0,
          VecBuilder.fill(0.01));
  private final PIDController pidController = new PIDController(1, 0, 0);
  private final ElevatorFeedforward climberFFModelUp =
      new ElevatorFeedforward(0.0, 0.0379, 5.85, 0.04); // TODO: tune
  private final ElevatorFeedforward climberFFModelDown =
      new ElevatorFeedforward(0, 0, 0); // TODO: tune

  private final ExponentialProfile climberProfile =
      new ExponentialProfile(
          ExponentialProfile.Constraints.fromCharacteristics(
              10, climberFFModelUp.kv, climberFFModelUp.ka));

  private final ExponentialProfile climberProfileDown =
      new ExponentialProfile(
          ExponentialProfile.Constraints.fromCharacteristics(
              10, climberFFModelDown.kv, climberFFModelDown.ka));

  private ExponentialProfile.State climberGoal;
  private ExponentialProfile.State climberSetpoint;

  public ClimberIOSim() {

    climberGoal = new ExponentialProfile.State(1.05, 0);
    climberSetpoint = new ExponentialProfile.State(1.05, 0);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    double currentVelocity = sim.getVelocityMetersPerSecond();
    double positionError = climberSetpoint.position - sim.getPositionMeters();

    if (positionError > 0) {
      climberSetpoint = climberProfile.calculate(0.02, climberSetpoint, climberGoal);
      appliedVolts = climberFFModelUp.calculate(currentVelocity, climberSetpoint.velocity, 0.02);
    } else {
      climberSetpoint = climberProfileDown.calculate(0.02, climberSetpoint, climberGoal);
      appliedVolts = climberFFModelDown.calculate(currentVelocity, climberSetpoint.velocity, 0.02);
    }

    double pidOutput = pidController.calculate(sim.getPositionMeters(), climberSetpoint.position);

    appliedVolts += pidOutput;
    sim.setInputVoltage(appliedVolts);
    sim.update(0.02);

    inputs.climberCurrentAmps = new double[] {sim.getCurrentDrawAmps()};
    inputs.climberSetpointPosition = climberSetpoint.position;
    inputs.climberLeftVelocityMetersPerSec = climberSetpoint.velocity;
    inputs.climberRightVelocityMetersPerSec = climberSetpoint.velocity;
  }

  @Override
  public void setPosition(double position) {
    climberGoal = new ExponentialProfile.State(position, 0);
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
    sim.setInputVoltage(appliedVolts);
  }

  @Override
  public void resetEncoder(double position) {
    sim.setInput(0);
  }
}
