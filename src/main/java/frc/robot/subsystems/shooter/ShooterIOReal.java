package frc.robot.subsystems.shooter;

import static com.revrobotics.CANSparkBase.ControlType.kVelocity;
import static com.revrobotics.CANSparkBase.IdleMode.kCoast;
import static edu.wpi.first.wpilibj.Timer.delay;
import static frc.robot.Constants.currentRobot;
import static frc.robot.util.SparkUtils.Data.*;
import static frc.robot.util.SparkUtils.Sensor.INTEGRATED;
import static frc.robot.util.SparkUtils.configureFrameStrategy;
import static java.lang.Math.abs;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import java.util.Set;

public class ShooterIOReal implements ShooterIO {
  private static final double topMotorkS =
      switch (currentRobot) {
        case CLEF -> 0.13;
        case LIGHTCYCLE -> 0.25;
      };
  private static final double bottomMotorkS =
      switch (currentRobot) {
        case CLEF -> 0.15;
        case LIGHTCYCLE -> 0.25;
      };

  private boolean openLoop = false;
  private final CANSparkMax topMotorLeader = new CANSparkMax(20, CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax topMotorFollower =
      new CANSparkMax(17, CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax bottomMotorLeader =
      new CANSparkMax(22, CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax bottomMotorFollower =
      new CANSparkMax(21, CANSparkMax.MotorType.kBrushless);

  private final SparkPIDController topMotorPID;
  private final SparkPIDController bottomMotorPID;

  private double topShooterSetpoint;
  private double bottomShooterSetpoint;

  public ShooterIOReal() {
    topMotorLeader.restoreFactoryDefaults();
    topMotorFollower.restoreFactoryDefaults();
    bottomMotorLeader.restoreFactoryDefaults();
    bottomMotorFollower.restoreFactoryDefaults();

    delay(0.25);

    topMotorLeader.setInverted(true);
    topMotorFollower.setInverted(true);
    bottomMotorLeader.setInverted(false);
    topMotorFollower.setInverted(false);

    topMotorLeader.enableVoltageCompensation(10.0);
    topMotorFollower.enableVoltageCompensation(10.0);
    bottomMotorLeader.enableVoltageCompensation(10.0);
    bottomMotorFollower.enableVoltageCompensation(10.0);

    // Tune acceptable current limit, don't want to use all power if shoot while moving
    topMotorLeader.setSmartCurrentLimit(50);
    topMotorFollower.setSmartCurrentLimit(50);
    bottomMotorLeader.setSmartCurrentLimit(50);
    bottomMotorFollower.setSmartCurrentLimit(50);

    topMotorLeader.getEncoder().setVelocityConversionFactor(2);
    topMotorFollower.getEncoder().setVelocityConversionFactor(2);
    bottomMotorLeader.getEncoder().setVelocityConversionFactor(2);
    bottomMotorFollower.getEncoder().setVelocityConversionFactor(2);

    topMotorLeader.setIdleMode(kCoast);
    topMotorFollower.setIdleMode(kCoast);
    bottomMotorLeader.setIdleMode(kCoast);
    bottomMotorFollower.setIdleMode(kCoast);

    configureFrameStrategy(
        topMotorLeader,
        Set.of(POSITION, VELOCITY, CURRENT, INPUT_VOLTAGE, APPLIED_OUTPUT),
        Set.of(INTEGRATED),
        true);

    configureFrameStrategy(topMotorFollower, Set.of(CURRENT), Set.of(INTEGRATED), false);

    configureFrameStrategy(
        bottomMotorLeader,
        Set.of(POSITION, VELOCITY, CURRENT, INPUT_VOLTAGE, APPLIED_OUTPUT),
        Set.of(INTEGRATED),
        true);

    configureFrameStrategy(bottomMotorFollower, Set.of(CURRENT), Set.of(INTEGRATED), false);
    configureFrameStrategy(
        topMotorLeader,
        Set.of(POSITION, VELOCITY, CURRENT, INPUT_VOLTAGE, APPLIED_OUTPUT),
        Set.of(INTEGRATED),
        true);

    configureFrameStrategy(topMotorFollower, Set.of(CURRENT), Set.of(INTEGRATED), false);

    configureFrameStrategy(
        bottomMotorLeader,
        Set.of(POSITION, VELOCITY, CURRENT, INPUT_VOLTAGE, APPLIED_OUTPUT),
        Set.of(INTEGRATED),
        true);

    configureFrameStrategy(bottomMotorFollower, Set.of(CURRENT), Set.of(INTEGRATED), false);

    topMotorLeader.getEncoder().setMeasurementPeriod(10);
    topMotorFollower.getEncoder().setMeasurementPeriod(10);
    bottomMotorLeader.getEncoder().setMeasurementPeriod(10);
    bottomMotorFollower.getEncoder().setMeasurementPeriod(10);

    topMotorLeader.getEncoder().setAverageDepth(4);
    topMotorFollower.getEncoder().setAverageDepth(4);
    bottomMotorLeader.getEncoder().setAverageDepth(4);
    bottomMotorFollower.getEncoder().setAverageDepth(4);
    // These filtering settings have visible jitter, but the blarg
    // says its ok because controls will smooth it out

    topMotorPID = topMotorLeader.getPIDController();
    bottomMotorPID = bottomMotorLeader.getPIDController();

    topMotorPID.setP(1e-4); // TODO tune
    bottomMotorPID.setP(1e-4);

    topMotorPID.setFF(
        switch (currentRobot) {
          case CLEF -> 1.05 * 0.0010967 / 10.0;
          case LIGHTCYCLE -> 0.98 * 0.001137 / 10.0;
        }); // Divide by 10 because of voltage compensation
    bottomMotorPID.setFF(
        switch (currentRobot) {
          case CLEF -> 1.04 * 0.0010492 / 10.0;
          case LIGHTCYCLE -> 0.98 * 0.001137 / 10.0;
        });

    topMotorFollower.follow(topMotorLeader, true);
    bottomMotorFollower.follow(bottomMotorLeader, true);
  }

  /** Updates the set of loggable inputs. */
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.shooterAppliedVolts =
        new double[] {
          topMotorLeader.getAppliedOutput() * topMotorLeader.getBusVoltage(),
          bottomMotorLeader.getAppliedOutput() * bottomMotorLeader.getBusVoltage()
        };
    inputs.shooterCurrentAmps =
        new double[] {
          topMotorLeader.getOutputCurrent(),
          topMotorFollower.getOutputCurrent(),
          bottomMotorLeader.getOutputCurrent(),
          bottomMotorFollower.getOutputCurrent()
        };
    inputs.shooterVelocityRPM =
        new double[] {
          topMotorLeader.getEncoder().getVelocity(), bottomMotorLeader.getEncoder().getVelocity()
        };
    inputs.openLoopStatus = openLoop;
    inputs.topShooterSetpoint = topShooterSetpoint;
    inputs.bottomShooterSetpoint = bottomShooterSetpoint;
    inputs.atShootSpeed =
        abs(topShooterSetpoint - inputs.shooterVelocityRPM[0]) < 250
            && abs(bottomShooterSetpoint - inputs.shooterVelocityRPM[1]) < 500
            && (topShooterSetpoint > 2000 || bottomShooterSetpoint > 2000);
  }

  /** Run open loop at the specified voltage. Primarily for characterization. */
  public void setVoltage(double volts) {
    openLoop = true;
    topMotorLeader.setVoltage(volts);
    bottomMotorLeader.setVoltage(volts);
  }

  /** Run closed loop at the specified velocity. */
  public void setVelocity(double velocityRPM) {
    openLoop = false;
    topShooterSetpoint = velocityRPM * 1.05;
    bottomShooterSetpoint = velocityRPM;
    // Use FF (kV) + PID on-smax, arbFF pass in kS to linearize system, kA unnecessary, low inertia
    topMotorPID.setReference(topShooterSetpoint, kVelocity, 0, topMotorkS, ArbFFUnits.kVoltage);
    bottomMotorPID.setReference(
        bottomShooterSetpoint, kVelocity, 0, bottomMotorkS, ArbFFUnits.kVoltage);
  }

  public void setVelocity(double topRPM, double bottomRPM) {
    openLoop = false;
    topShooterSetpoint = topRPM;
    bottomShooterSetpoint = bottomRPM;
    // Use FF (kV) + PID on-smax, arbFF pass in kS to linearize system, kA unnecessary, low inertia
    topMotorPID.setReference(topShooterSetpoint, kVelocity, 0, topMotorkS, ArbFFUnits.kVoltage);
    bottomMotorPID.setReference(
        bottomShooterSetpoint, kVelocity, 0, bottomMotorkS, ArbFFUnits.kVoltage);
  }

  /** Stop in open loop. */
  public void stop() {
    openLoop = true;
    setVoltage(0.0);
  }
}
