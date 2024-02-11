package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ClimberConstants.*;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  public static final double GEAR_RATIO = 15.0 / 1.0;
  public static final double PULLEY_RADIUS = Units.inchesToMeters(1.0);

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private final SysIdRoutine sysId;

  private boolean activateExtendPID = false;
  private boolean extendZeroed = false;

  ElevatorFeedforward ffUp = new ElevatorFeedforward(1, 1, 1);
  ElevatorFeedforward ffDown = new ElevatorFeedforward(1, 1, 1);
  PIDController pidExtend = new PIDController(120, 0, 2); // tune needed

  public Climber(ClimberIO io) {
    this.io = io;

    pidExtend.setSetpoint(0);
    pidExtend.setTolerance(0.01);

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

  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  public void stopExtend() {
    io.stop();
  }

  public double getCharacterizationVelocity() {
    return inputs.climberAbsoluteVelocityMetersPerSec;
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  public void setExtendMeter(double positionMeters) {
    pidExtend.setSetpoint(positionMeters);
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
