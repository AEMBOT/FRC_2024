package frc.robot.subsystems.climber;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
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
    public static final double GEAR_RATIO  = 15.0/1.0;
    public static final double PULLEY_RADIUS = Units.inchesToMeters(1.0);

    private final ClimberIO io; 
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
    private final SysIdRoutine sysId;

    private boolean activateExtendPID = false;
    private boolean extendZeroed = false;

    ElevatorFeedforward ffUp = new ElevatorFeedforward(1,1,1);
    ElevatorFeedforward ffDown = new ElevatorFeedforward(1,1,1);    
    PIDController pidExtend = new PIDController(120, 0, 2); //tune needed

    public Climber(ClimberIO io){
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
        return Units.radiansPerSecondToRotationsPerMinute(inputs.climberAbsoluteVelocityMetersPerSec);
    }

    /** Returns the current velocity in radians per second. */
    public double getCharacterizationVelocity() {
        return inputs.climberAbsoluteVelocityMetersPerSec;
    }

    public void setExtendMeter(double positionMeters){
        pidExtend.setSetpoint(positionMeters);
    }
}