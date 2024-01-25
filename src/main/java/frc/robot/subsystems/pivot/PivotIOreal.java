import com.revrobotics.AbsoluteEncoder;

//has to rotate up and down
//stay in fixed angle
//uses 2 neo motors (opposite sides) with sparkmax controlers
//Absolute encoder
//set current limit (low)
//flows w/controler, press button -> go to fixed angle 
//This subsystem uses two NEOs and a REV Throughbore absolute encoder wired to DIO.
public class PivotIOSparkMax implements PivotIO {
    private static final double GEAR_RATIO = 1.5;
  
    private final CANSparkMax leader = new CANSparkMax(10, MotorType.kBrushless);
    private final CANSparkMax follower = new CANSparkMax(11, MotorType.kBrushless);
    private final RelativeEncoder encoder = leader.getEncoder();
    private final AbsoluteEncoder absoluteEncoder = new AbsoluteEncoder();

    public void updateInputs(PivotIOInputs inputs) {
        inputs.positionRad = Units.rotationsToRadians(absoluteEncoder.getPosition() / GEAR_RATIO);
        inputs.velocityRadPerSec =
            Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / GEAR_RATIO);
        inputs.appliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();
        inputs.currentAmps = new double[] {leader.getOutputCurrent(), follower.getOutputCurrent()};

    // Change naming of PivotIOInputs to match FlywheelIoInputs, don't need to re-specify pivot in variable name.
        
    public double pivotAbsolutePositionRad = 0.0;
    public double pivotAbsoluteVelocityRadPerSec = 0.0;
    public double pivotAppliedVolts = 0.0;
    public double[] pivotCurrentAmps =
        new double[] {}; // Log motors individually, useful for failure analysis
  }

  /** Updates the set of loggable inputs