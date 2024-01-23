package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  private final double pivotkS = 0.1;
  private final double pivotkG = 0.24;
  private final double pivotkV = 5.85;
  private final double pivotkA = 0.02;

  private final ArmFeedforward pivotFFModel;
  private final ExponentialProfile pivotProfile =
      new ExponentialProfile(
          ExponentialProfile.Constraints.fromCharacteristics(10, pivotkV, pivotkA));
  private ExponentialProfile.State pivotGoal = new ExponentialProfile.State();
  private ExponentialProfile.State pivotSetpoint = new ExponentialProfile.State();

  private final SysIdRoutine sysId;

  public Pivot(PivotIO io) {
    this.io = io;

    pivotFFModel = new ArmFeedforward(0.1, 0.24, 5.85, 0.02); // Recalc estimate, TODO characterize
    io.configurePID(0.1, 0.0, 0.02); // TODO characterize

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Flywheel/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    double currentVel = pivotSetpoint.velocity;
    pivotSetpoint = pivotProfile.calculate(0.02, pivotSetpoint, pivotGoal);
    io.setPosition(
        pivotSetpoint.position,
        pivotFFModel.calculate(
            pivotSetpoint.position,
            pivotSetpoint.velocity,
            (pivotSetpoint.velocity - currentVel) / 0.02));
  }

  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  public void runPosition(double positionRad) {
    pivotGoal = new ExponentialProfile.State(positionRad, 0);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }
}
