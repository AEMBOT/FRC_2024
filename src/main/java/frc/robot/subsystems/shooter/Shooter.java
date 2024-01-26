package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final SimpleMotorFeedforward shooterFFModel; // FF + P optimal flywheel control, no I/D
  private final PIDController shooterPIDController; // Low inertia system, use onboard PID?

  public Shooter(ShooterIO io) {
    this.io = io;

    shooterFFModel =
        new SimpleMotorFeedforward(0.1, 0.26, 0.18); // Recalc estimate, TODO characterize
    shooterPIDController = new PIDController(0.1, 0.0, 0.0); // TODO characterize
  }

  @Override
  public void periodic() {}
}
