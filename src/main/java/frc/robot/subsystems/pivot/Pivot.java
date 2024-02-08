package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Volts;
import static java.lang.Math.abs;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
  private final SysIdRoutine sysId;

  public Pivot(PivotIO io) {
    this.io = io;

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
    io.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);
  }

  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  public void runPosition(double positionRad) {
    Logger.recordOutput("Pivot/GoalRad", positionRad);
    io.setPosition(positionRad);
  }

  public Command goToAngle(double positionRad) {
    // TODO make sure 0.02 radian tolerance is achievable
    return run(() -> runPosition(positionRad))
        .until(() -> abs(inputs.pivotAbsolutePositionRad - inputs.pivotGoalPosition) < 0.02);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  public Pose3d getPose3D() {
    return new Pose3d(-0.2, 0, 0.255, new Rotation3d(0, -inputs.pivotAbsolutePositionRad, 0));
  }
}
