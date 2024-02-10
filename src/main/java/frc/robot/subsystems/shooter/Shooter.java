package frc.robot.subsystems.shooter;

import static frc.robot.Constants.ShooterConstants.shooterIdleRPM;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) {
    this.io = io;
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

  private void setVelocityRPM(double velRPM) {
    io.setVelocity(velRPM);
  }

  // We're using this because the overhead of the Java Stream API sucks
  private double findMin(double[] array) {
    if (array == null || array.length == 0) return 0;
    if (array.length == 1) return array[0];
    else return Math.min(array[0], array[1]);
  }
}
