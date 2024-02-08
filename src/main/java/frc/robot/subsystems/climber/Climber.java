package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ClimberConstants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private final SysIdRoutine sysId;

  private final CANSparkMax m_winchMotorRight =
      new CANSparkMax(winchMotorRightCanID, MotorType.kBrushless);
  private final CANSparkMax m_winchMotorLeft =
      new CANSparkMax(winchMotorLeftCanID, MotorType.kBrushless);

  public RelativeEncoder rightEncoder = m_winchMotorRight.getEncoder();
  public RelativeEncoder leftEncoder = m_winchMotorLeft.getEncoder();
  private boolean activateExtendPID = false;
  private boolean extendZeroed = false;

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

  /** Returns the current velocity in RPM. */
  @AutoLogOutput
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.climberAbsoluteVelocityRadPerSec);
  }

  /** Returns the current velocity in radians per second. */
  public double getCharacterizationVelocity() {
    return inputs.climberAbsoluteVelocityRadPerSec;
  }

  public void resetRightEncoder() {
    rightEncoder.setPosition(0);
  }

  public void resetLeftEncoder() {
    leftEncoder.setPosition(0);
  }

  public void setExtendMeter(double positionMeters) {
    pidExtend.setSetpoint(positionMeters);
  }
}
