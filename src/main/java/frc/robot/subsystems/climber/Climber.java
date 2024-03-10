package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ClimberConstants.*;
import static java.lang.Math.abs;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private final SysIdRoutine sysId;

  public Climber(ClimberIO io) {
    this.io = io;

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Climber/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  @AutoLogOutput
  public boolean atGoal() {
    return abs(inputs.climberLeftPositionMeters - inputs.climberSetpointPosition) < 0.02;
  }

  private void runVolts(double volts) {
    io.setVoltage(volts);
  }

  private void setPosition(double position) {
    io.setPosition(position);
  }

  private void setPositionClimbing(double position) {
    io.setPositionClimbing(position);
  }

  public Command runVoltsCommand(double voltage) {
    return run(() -> runVolts(voltage)).finallyDo(() -> runVolts(0.0));
  }

  public Command setPositionCommand(double targetPosition) {
    return run(() -> setPosition(targetPosition)).finallyDo(() -> io.setVoltage(0.0));
  }

  public Command setPositiionCommandClimbing(double targetPosition) {
    return run(() -> setPositionClimbing(targetPosition)).finallyDo(() -> io.setVoltage(0));
  }

  public Command getHomingCommand() {
    return Commands.sequence(
        runOnce(() -> io.setHoming(true)),
        runVoltsCommand(-2.0).until(io::isCurrentLimited),
        runOnce(io::resetEncoder),
        runOnce(() -> io.setHoming(false)));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  public double getCurrentLimit(boolean homingBool) {
    // homingBool true if we are in homing mode
    if (homingBool) {
      return homingCurrentLimit;
    } else {
      return extendCurrentLimit;
    }
  }
}
