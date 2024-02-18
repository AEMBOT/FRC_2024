package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.UPDATE_PERIOD;
import static java.lang.Math.abs;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
  private final SysIdRoutine sysId;

  public Pivot(PivotIO io) {
    this.io = io;

    new Trigger(() -> inputs.openLoopStatus).onFalse(runOnce(io::resetExponentialProfile));

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.2).per(Seconds.of(1)),
                Volts.of(8),
                Seconds.of(30),
                (state) -> Logger.recordOutput("Flywheel/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);
  }

  @AutoLogOutput
  public boolean inHandoffZone() {
    return inputs.pivotAbsolutePositionRad < Units.degreesToRadians(80);
  }

  public boolean atGoal() {
    return abs(inputs.pivotAbsolutePositionRad - inputs.pivotGoalPosition) < 0.02;
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

  public Command runVoltsCommand(double volts) {
    return run(() -> runVolts(volts)).finallyDo(() -> runVolts(0.0));
  }

  public Command changeGoalPosition(double velocityRadPerSec) {
    return setPositionCommand(() -> inputs.pivotGoalPosition + (velocityRadPerSec * UPDATE_PERIOD))
        .finallyDo(io::resetExponentialProfile);
  }

  public Command setPositionCommand(DoubleSupplier posRad) {
    return run(() -> runPosition(posRad.getAsDouble()));
  }

  public Command getDefault() {
    // return setPositionCommand(() -> Units.degreesToRadians(40));
    return defer(
        () -> {
          double holdPos = inputs.pivotAbsolutePositionRad;
          return setPositionCommand(() -> holdPos);
        });
  }

  // These functions should only be accessed through command mutexing, hence private
  private void runVolts(double volts) {
    io.setVoltage(volts);
  }

  private void runPosition(double positionRad) {
    Logger.recordOutput("Pivot/GoalRad", positionRad);
    io.setPosition(positionRad);
  }
}
