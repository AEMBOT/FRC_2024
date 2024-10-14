package frc.robot.subsystems.pivot;

import static edu.wpi.first.math.MathUtil.clamp;
import static edu.wpi.first.wpilibj.Timer.delay;
import static frc.robot.Constants.PivotConstants.PIVOT_MAX_POS_RAD;
import static frc.robot.Constants.PivotConstants.PIVOT_MIN_POS_RAD;
import static frc.robot.Constants.Robot.CLEF;
import static frc.robot.Constants.Robot.LIGHTCYCLE;
import static frc.robot.Constants.currentRobot;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class PivotIOReal implements PivotIO {

  private boolean openLoop = true;
  private static final double GEAR_RATIO =
      switch (currentRobot) {
        case CLEF -> 5 * 4 * (42.0 / 9.0);
        case LIGHTCYCLE -> 4 * 3 * 3 * (42.0 / 9.0);
      };
  private double LAST_TIME = 0.0;

  private final CANSparkMax motorLeader = new CANSparkMax(11, MotorType.kBrushless);
  private final CANSparkMax motorFollower = new CANSparkMax(10, MotorType.kBrushless);
  private final DutyCycleEncoder encoder = new DutyCycleEncoder(3);
  private final ArmFeedforward pivotFFModel =
      switch (currentRobot) {
        case CLEF -> new ArmFeedforward(0.35, 0.35, 1.79, 0.3);
        case LIGHTCYCLE -> new ArmFeedforward(0.20, 0.17, 3.27, 0.295); // SysID Kv 2.683
      };

  private final PIDController pidController =
      switch (currentRobot) {
        case CLEF -> new PIDController(12, 0, 0.00);
        case LIGHTCYCLE -> new PIDController(35, 0, 0.1);
      };

  private final TrapezoidProfile pivotProfile =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(2, 5));
  private TrapezoidProfile.State pivotGoal;
  private TrapezoidProfile.State pivotSetpoint;

  public PivotIOReal() {
    motorLeader.restoreFactoryDefaults();
    motorFollower.restoreFactoryDefaults();

    delay(0.25);

    motorLeader.setIdleMode(CANSparkBase.IdleMode.kBrake);
    motorFollower.setIdleMode(CANSparkBase.IdleMode.kBrake);

    // I have no clue what's going on with invert settings, but both inverted and invert follow
    // works
    motorLeader.setInverted(true);
    motorFollower.setInverted(true);

    motorLeader.setSmartCurrentLimit(60);
    motorFollower.setSmartCurrentLimit(60);

    motorLeader.getEncoder().setVelocityConversionFactor(((2 * Math.PI) / 60) / GEAR_RATIO);

    delay(0.25);

    motorFollower.follow(motorLeader, true);

    encoder.setPositionOffset(
        switch (currentRobot) {
          case CLEF -> 4.04433682 / (2 * Math.PI);
          case LIGHTCYCLE -> 0.1974 / (2 * Math.PI);
        }); // Convert from offset rads to offset rotations

    while (getAbsoluteEncoderPosition() < 0.1 || getAbsoluteEncoderPosition() > 3) {
      DriverStation.reportError(
          "The pivot encoder is reporting an invalid position. Is it plugged in?", false);
      delay(1);
    }
    pivotGoal = new TrapezoidProfile.State(getAbsoluteEncoderPosition(), 0);
    pivotSetpoint = new TrapezoidProfile.State(getAbsoluteEncoderPosition(), 0);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.pivotAbsolutePositionRad = getAbsoluteEncoderPosition();
    inputs.pivotAppliedVolts = motorLeader.getAppliedOutput() * motorLeader.getBusVoltage();
    inputs.pivotCurrentAmps =
        new double[] {motorLeader.getOutputCurrent(), motorFollower.getOutputCurrent()};
    inputs.pivotAbsoluteVelocityRadPerSec = motorLeader.getEncoder().getVelocity();
    inputs.pivotGoalPosition = pivotGoal.position;
    inputs.pivotSetpointPosition = pivotSetpoint.position;
    inputs.pivotSetpointVelocity = pivotSetpoint.velocity;
    inputs.openLoopStatus = openLoop;
  }

  /** Sets the angle of the pivot, in radians. */
  @Override
  public void setPosition(double positionRad) {
    openLoop = false;
    pivotGoal = new TrapezoidProfile.State(positionRad, 0);
    double currentVelocity = pivotSetpoint.velocity;
    pivotSetpoint =
        pivotProfile.calculate(
            // If new profile starting 0.02
            // Else make sure profile is moving at actual loop time
            (Timer.getFPGATimestamp() - LAST_TIME > 0.25)
                ? (Timer.getFPGATimestamp() - LAST_TIME)
                : 0.02,
            pivotSetpoint,
            pivotGoal);
    double feedForward =
        //        pivotFFModel.calculate(
        //            pivotSetpoint.position,
        //            pivotSetpoint.velocity,
        //            (pivotSetpoint.velocity - currentVelocity) / UPDATE_PERIOD);
        pivotFFModel.calculate(pivotGoal.position, 0);
    double pidOutput = pidController.calculate(getAbsoluteEncoderPosition(), pivotGoal.position);

    Logger.recordOutput("Pivot/CalculatedFFVolts", feedForward);
    Logger.recordOutput("Pivot/PIDCommandVolts", pidOutput);

    setMotorVoltage(feedForward + pidOutput);

    LAST_TIME = Timer.getFPGATimestamp();
  }

  /** Run open loop at the specified voltage. */
  @Override
  public void setVoltage(double volts) {
    openLoop = true;
    setMotorVoltage(volts);
  }

  /** Stop in open loop. */
  @Override
  public void stop() {
    openLoop = true;
    setVoltage(0.0);
  }

  private double getAbsoluteEncoderPosition() {
    return switch (currentRobot) {
      case CLEF -> Units.rotationsToRadians(
          encoder.getAbsolutePosition() - encoder.getPositionOffset());
      case LIGHTCYCLE -> Units.rotationsToRadians(
          -encoder.getAbsolutePosition() + 1 - encoder.getPositionOffset());
    };
  }

  private void setMotorVoltage(double volts) {
    if (getAbsoluteEncoderPosition() < PIVOT_MIN_POS_RAD) {
      volts = clamp(volts, -1, Double.MAX_VALUE);
    }
    if (getAbsoluteEncoderPosition() > PIVOT_MAX_POS_RAD) {
      volts = clamp(volts, -Double.MAX_VALUE, 1);
    }

    motorLeader.setVoltage(volts);
  }

  @Override
  public void resetExponentialProfile() {
    pivotGoal = new TrapezoidProfile.State(getAbsoluteEncoderPosition(), 0);
    pivotSetpoint = new TrapezoidProfile.State(getAbsoluteEncoderPosition(), 0);
  }
}
