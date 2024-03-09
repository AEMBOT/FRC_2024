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
import java.util.Optional;
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
    Logger.recordOutput(
        "Shooter/Running Command",
        Optional.ofNullable(this.getCurrentCommand()).map(Command::getName).orElse("None"));
  }

  public boolean isAtShootSpeed() {
    return inputs.atShootSpeed;
  }

  public Command getDefault() {
    // If the shooter was running fast and is now coasting down,
    // we don't want to force the speed down-- preserve momentum
    //    return Commands.waitUntil(
    //            () -> {
    //              Logger.recordOutput("shooter min velocity", findMin(inputs.shooterVelocityRPM));
    //              return findMin(inputs.shooterVelocityRPM) < shooterIdleRPM;
    //            })
    //        .andThen(run(() -> setVelocityRPM(shooterIdleRPM)));

    return Commands.waitUntil(
            () -> {
              Logger.recordOutput("shooter min velocity", findMin(inputs.shooterVelocityRPM));
              return findMin(inputs.shooterVelocityRPM) < shooterIdleRPM;
            })
        .andThen(setVoltageCommand(0.5));
  }

  public Command setVelocityRPMCommand(double velRPM) {
    // The finallyDo shooter stop sets voltage to 0, which lets the motor coast down
    // without unintended latent power application from PID still running
    // after the run is interrupted (ex by letting go of button)
    return run(() -> setVelocityRPM(velRPM)).finallyDo(io::stop);
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

  public Command runShooterCharacterization() {
    return Commands.sequence(
        sysId.quasistatic(kForward),
        stopCommand(),
        Commands.waitSeconds(2.0),
        sysId.quasistatic(kReverse),
        stopCommand(),
        Commands.waitSeconds(2.0),
        sysId.dynamic(kForward),
        stopCommand(),
        Commands.waitSeconds(2.0),
        sysId.dynamic(kReverse));
  }
}
