package frc.robot.subsystems.shooter;

import static frc.robot.Constants.ShooterConstants.shooterIdleRPM;

import edu.wpi.first.wpilibj2.command.Command;
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
    return run(() -> setVelocityRPM(shooterIdleRPM));
  }

  public Command setVelocityRPMCommand(double velRPM) {
    return run(() -> setVelocityRPM(velRPM));
  }

  private void setVelocityRPM(double velRPM) {
    io.setVelocity(velRPM);
  }
}
