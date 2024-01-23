package frc.robot.subsystems.pivot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {
  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  private final ArmFeedforward pivotFFModel;
  private final PIDController pivotPIDController;

  public Pivot(PivotIO io) {
    this.io = io;

    pivotFFModel = new ArmFeedforward(0.1, 0.24, 5.85, 0.02); // Recalc estimate, TODO characterize
    pivotPIDController = new PIDController(0.1, 0.0, 0.02); // TODO characterize
  }
}
