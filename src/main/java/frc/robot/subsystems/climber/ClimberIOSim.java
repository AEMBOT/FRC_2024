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
  private static final double GEAR_RATIO = 1.5;
  private double appliedVoltsUp = 0;
  private double appliedVoltsDown = 0;
  private boolean UpDirection = true; // boolean for climber going up or down

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
    double currentVelocity = climberSetpoint.velocity;
    // change 0.02 for constants.update period when merged properly in climbersetpoint definitionand
    // feedforwardup definition
    climberSetpoint = climberProfile.calculate(0.02, climberSetpoint, climberGoal);
    double feedForwardUp =
        climberFFModelUp.calculate(
            climberSetpoint.position,
            climberSetpoint.velocity,
            (climberSetpoint.velocity - currentVelocity) / 0.02);
    double feedForwardDown =
        climberFFModelDown.calculate(
            climberSetpoint.position,
            climberSetpoint.velocity,
            (climberSetpoint.velocity - currentVelocity) / 0.02);
    double pidOutput = pidController.calculate(sim.getPositionMeters(), climberSetpoint.position);

    sim.setInputVoltage(appliedVoltsUp);
    sim.update(0.02);

    appliedVoltsUp = feedForwardUp + pidOutput;
    appliedVoltsDown = feedForwardDown; // TODO: figure out if need pid for down or not

    if (UpDirection) {
      sim.setInputVoltage(appliedVoltsUp);
    } else {
      sim.setInputVoltage(appliedVoltsDown);
    }

    inputs.climberAbsoluteVelocityMetersPerSec = sim.getVelocityMetersPerSecond();
    inputs.climberAppliedVoltsUp = appliedVoltsUp;
    inputs.climberAppliedVoltsDown = appliedVoltsDown;
    inputs.climberCurrentAmps = new double[] {sim.getCurrentDrawAmps()};
    inputs.climberGoalPosition = climberGoal.position;
    inputs.climberSetpointPosition = climberSetpoint.position;
    inputs.climberSetpointVelocity = climberSetpoint.velocity;
    inputs.upDirectionStatus = UpDirection;
  }

  @Override
  public void setPosition(double position) {
    climberGoal = new ExponentialProfile.State(position, 0);
  }

  @Override
  public void setVoltage(double volts) {
    appliedVoltsDown = volts;
    appliedVoltsUp = volts;
    sim.setInputVoltage(appliedVoltsUp);
  }

  @Override
  public void resetEncoder(double position) {
    // this does not actually reset the encoder, but I need to do it for the file to inherit
    // climberio
    sim.setInput(0);
  }

  @Override
  public void stop() {
    setVoltage(0);
  }
}
