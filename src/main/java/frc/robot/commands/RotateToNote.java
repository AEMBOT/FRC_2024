package frc.robot.commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.LogTable;

//this won't use advantage kit logging because it relies on network tables


public class RotateToNote extends Command {
  private Drive drive;
  private boolean slowMode;

  private String notePosition;
  private double notePositionNum;
  private String previousNotePosition;
  private double previousNotePositionNum;
  private final PIDController thetaController = new PIDController(0,0,0); // change this
  private double xRot;
  private double yRot;
  private double omega;

  private boolean running = false;
  private double thetaErrorAbs;
  private final SlewRateLimiter thetaRateLimiter = new SlewRateLimiter(5);

  //call network table stuff in robot container

  public RotateToNote(Drive drive, boolean slowMode, String notePosition){
    this.drive = drive;
    this.slowMode = slowMode; 
    this.notePosition = notePosition;
  }

  public RotateToNote(Drive drive, boolean slowMode, Double notePostionNum){
    this.drive = drive;
    this.slowMode = slowMode;
    this.notePositionNum = notePostionNum;
  }

  @Override
  public void initialize(){
    thetaRateLimiter.reset(0);
    previousNotePosition = SmartDashboard.getString("note position", "Center");
    previousNotePositionNum = SmartDashboard.getNumber("note x pixel", 0);
    //previousNotePosition = Logger.recordOutput("Note Position", previousNotePosition);
    //previousNotePositionNum = Logger.recordOutput("Note Position", previousNotePositionNum);
  }
  
  @Override
  public void execute(){
    //IMPORTANT: MAKE SURE THE LEFT, CENTER, RIGHT VALUES ARE CORRECT WITH THE X PIXEL VALUES
    //if xpixel is less than 0 then go until range 
    //if xpixel is greater than 0 then go until range
    running = true;
    notePosition = SmartDashboard.getString("note position", "Center");
    notePositionNum = SmartDashboard.getNumber("note x pixel", 0);
    if (notePosition != "Center" && notePositionNum > -5 && notePositionNum < 5){
      //get better values for notepositionnum where its in the center
      if (notePosition == "Right" && notePositionNum > 30){
        xRot = 15;
        yRot = 15;
      }
      else if (notePosition == "Left" && notePositionNum < 20){
        xRot = -15;
        yRot = -15;
      }
      else{
        xRot = 0;
        yRot =0;
      }

    omega = Math.copySign(Math.sqrt(xRot * xRot + yRot * yRot), xRot);

    Rotation2d linearDirection =
              new Rotation2d(xRot, yRot);

    Translation2d linearVelocity =
      new Pose2d(new Translation2d(), linearDirection)
          .transformBy(new Transform2d(0, 0.0, new Rotation2d()))
          .getTranslation();
    boolean isFlipped =
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    drive.runVelocity(
      ChassisSpeeds.fromFieldRelativeSpeeds(
          0,
          0,
          omega * drive.getMaxAngularSpeedRadPerSec(),
              isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation())); 
  } 
  
  
}}