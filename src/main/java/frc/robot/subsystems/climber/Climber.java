package frc.robot.subsystems.climber;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ClimberConstants.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Volts;

public class Climber extends SubsystemBase{
    private final ClimberIO io; 
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
    private final SysIdRoutine sysId;

    private final CANSparkMax m_winchMotorRight = new CANSparkMax(winchMotorRightCanID, MotorType.kBrushless);
    private final CANSparkMax m_winchMotorLeft = new CANSparkMax(winchMotorLeftCanID, MotorType.kBrushless);

    public RelativeEncoder rightEncoder = m_winchMotorRight.getEncoder();
    public RelativeEncoder leftEncoder = m_winchMotorLeft.getEncoder();
    private boolean activateExtendPID = false;
    private boolean extendZeroed = false;

    PIDController pidExtend = new PIDController(120, 0, 2); //tune needed

    

    public Climber(ClimberIO io){
        this.io = io;
        m_winchMotorRight.restoreFactoryDefaults();
        m_winchMotorLeft.restoreFactoryDefaults();
        
        m_winchMotorRight.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_winchMotorLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);

        m_winchMotorRight.setSmartCurrentLimit(extendCurrentLimit);
        m_winchMotorLeft.setSmartCurrentLimit(extendCurrentLimit); //make sure to have logic for homing current limit

        m_winchMotorRight.setInverted(false);
        m_winchMotorLeft.setInverted(false);

        m_winchMotorRight.setSmartCurrentLimit(extendCurrentLimit);
        m_winchMotorLeft.setSmartCurrentLimit(extendCurrentLimit);

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
        double pidExtendValueRight = pidExtend.calculate(rightEncoder.getPosition());
        double pidExtendValueLeft = pidExtend.calculate(leftEncoder.getPosition()); 
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


    public void resetRightEncoder(){
        rightEncoder.setPosition(0);
    }

    public void resetLeftEncoder(){
        leftEncoder.setPosition(0);
    }

    public void setExtendMeter(double positionMeters){
        pidExtend.setSetpoint(positionMeters);
    }
}