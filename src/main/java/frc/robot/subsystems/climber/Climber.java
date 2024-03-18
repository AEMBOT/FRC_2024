package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ClimberConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.Optional;
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
    Logger.recordOutput(
        "Climber/Running Command",
        Optional.ofNullable(this.getCurrentCommand()).map(Command::getName).orElse("None"));
  }

  private void runVolts(double volts) {
    io.setVoltage(volts);
  }

  private void setPosition(double position) {
    io.setPosition(position);
  }

  public Command runVoltsCommand(double voltage) {
    return run(() -> runVolts(voltage)).finallyDo(() -> runVolts(0.0));
  }

  public Command setPositionCommand(double targetPosition) {
    return run(() -> setPosition(targetPosition)).finallyDo(() -> io.setVoltage(0.0));
  }

  public Command getHomingCommand() {
    return Commands.sequence(
        runOnce(() -> io.setHoming(true)),
        Commands.parallel(
            Commands.run(() -> io.setLeftVoltage(-2.0))
                .until(io::isLeftCurrentLimited)
                .finallyDo(() -> io.setLeftVoltage(0.0)),
            Commands.run(() -> io.setRightVoltage(-2.0))
                .until(io::isRightCurrentLimited)
                .finallyDo(() -> io.setRightVoltage(0.0))),
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

  public Command setLeftPositionCommand(double position) {
    return runOnce(() -> io.setLeftPosition(position));
  }

  public Command setRightPositionCommand(double position) {
    return runOnce(() -> io.setRightPosition(position));
  }
}
