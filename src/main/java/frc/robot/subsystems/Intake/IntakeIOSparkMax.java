package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants;

public class IntakeIOSparkMax implements IntakeIO {
  private final CANSparkMax intakeMotorTop =
      new CANSparkMax(
          Constants.IntakeConstants.intakeMotorPortTop, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax intakeMotorBottom =
      new CANSparkMax(
          Constants.IntakeConstants.intakeMotorPortBottom, CANSparkLowLevel.MotorType.kBrushless);
    public IntakeIOSparkMax(){
        intakeMotorBottom.setInverted(true);
    }
  @Override
  public void SetVoltage(double voltage) {
    intakeMotorTop.setVoltage(voltage);
    intakeMotorBottom.setVoltage(voltage);
  }
}
