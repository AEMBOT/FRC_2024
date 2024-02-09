package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward;
import static edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse;
import static frc.robot.Constants.ShooterConstants.shooterIdleRPM;

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
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Shooter/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> io.setVoltage(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public Command getDefaultCommand() {
    // If the shooter was running fast and is now coasting down,
    // we don't want to force the speed down-- preserve momentum
    return Commands.waitUntil(() -> findMin(inputs.shooterVelocityRPM) < shooterIdleRPM)
        .andThen(run(() -> setVelocityRPM(shooterIdleRPM)));
  }

  public Command setVelocityRPMCommand(double velRPM) {
    return run(() -> setVelocityRPM(velRPM));
  }

  public Command stopCommand() {
    return run(() -> io.setVoltage(0.0));
  }

  private void setVelocityRPM(double velRPM) {
    io.setVelocity(velRPM);
  }

  // We're using this because the overhead of the Java Stream API sucks
  private double findMin(double[] array) {
    if (array == null || array.length == 0) return 0;
    if (array.length == 1) return array[0];
    else return Math.min(array[0], array[1]);
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
