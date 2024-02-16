package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward;
import static edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse;
import static frc.robot.Constants.ShooterConstants.shooterIdleRPM;
import static java.lang.Math.abs;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private final SysIdRoutine sysId;

  public Shooter(ShooterIO io) {
    this.io = io;

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Seconds.of(1)),
                Volts.of(8),
                Seconds.of(16),
                (state) -> Logger.recordOutput("Shooter/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> io.setVoltage(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public Command getDefault() {
    // If the shooter was running fast and is now coasting down,
    // we don't want to force the speed down-- preserve momentum
    return Commands.waitUntil(() -> findMin(inputs.shooterVelocityRPM) < shooterIdleRPM)
        .andThen(run(() -> setVelocityRPM(shooterIdleRPM)));
  }

  public Command setVelocityRPMCommand(double velRPM) {
    return runOnce(() -> setVelocityRPM(velRPM));
  }

  public Command stopCommand() {
    return runOnce(() -> io.setVoltage(0.0));
  }

  public Command setVoltageCommand(double volts) {
    return run(() -> io.setVoltage(volts)).finallyDo(() -> io.setVoltage(0.0));
  }

  private void setVelocityRPM(double velRPM) {
    io.setVelocity(velRPM);
  }

  // We're using this because the overhead of the Java Stream API sucks
  private double findMin(double[] array) {
    if (array == null || array.length == 0) return 0;
    if (array.length == 1) return abs(array[0]);
    else return Math.min(abs(array[0]), abs(array[1]));
  }

  private Command sysIDStateSet(int val) {
    return runOnce(() -> inputs.sysIDState = val);
  }

  public Command runShooterCharacterization() {
    return Commands.sequence(
        sysIDStateSet(0),
        sysId.quasistatic(kForward),
        sysIDStateSet(1),
        stopCommand(),
        sysIDStateSet(2),
        Commands.waitSeconds(5.0),
        sysIDStateSet(3),
        sysId.quasistatic(kReverse),
        sysIDStateSet(4),
        stopCommand(),
        sysIDStateSet(5),
        Commands.waitSeconds(5.0),
        sysIDStateSet(6),
        sysId.dynamic(kForward).withTimeout(5.0),
        sysIDStateSet(7),
        stopCommand(),
        sysIDStateSet(8),
        Commands.waitSeconds(5.0),
        sysIDStateSet(9),
        sysId.dynamic(kReverse).withTimeout(5.0));
  }
}
