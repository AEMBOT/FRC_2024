package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;

public class RotateDriveToNote {

  private RotateDriveToNote() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("NotesDetection");
    StringSubscriber notePosChar = table.getStringTopic("note position").subscribe("center");
    DoubleSubscriber notePosX = table.getDoubleTopic("note x pixel").subscribe(0.0);
    DoubleSubscriber notePosY = table.getDoubleTopic("note y pixel").subscribe(0.0);
    DoubleArraySubscriber notePosYArray =
        table.getDoubleArrayTopic("note y pixel array").subscribe(new double[] {});
    DoubleArraySubscriber notePosXArray =
        table.getDoubleArrayTopic("note x pixel array").subscribe(new double[] {});

    // default values need to be center and center (like 50, 50 or something)
    // need to build in logic to take left, right, center of closest (y value) and/or xpixel based
    // on closest
    // first with left, center, right
    // sequential command for rotate and drive
    // build in logic for y values which to do first, track the one with the highest y value at all
    // times, when driving towards it,

  }

  public static Command RotatToNoteWithArray(
      Drive drive, double[] notePosXArray, double[] notePosYArray) {
    return Commands.run(
        () -> {
          double omega = 0;
          double minY = notePosYArray[0];
          int minYIndex = 0;
          for (int i = 1; i < notePosYArray.length; i++) {
            if (notePosYArray[i] < minY) {
              minY = notePosYArray[i];
              minYIndex = i;
            }
            if (notePosXArray[minYIndex] < 50) {
              omega = 20;
            } else if (notePosXArray[minYIndex] > 50) {
              omega = -20;
            } else {
              omega = 0;
            }
            drive.runVelocity(
                ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, omega * 0.3, drive.getRotation()));
          }
        });
  }

  public static Command RotateToNote(
      Drive drive, String notePosChar, double notePosX, double notePosY) {
    return Commands.run(
        () -> {
          double omega = 0;
          if (notePosChar == "Left" && notePosX < 50) {
            omega = 20;
          } else if (notePosChar == "Right" && notePosX > 50) {
            omega = -20;
          } else {
            omega = 0;
          }
          drive.runVelocity(
              ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, omega * 0.3, drive.getRotation()));
        });
  }

  public static Command DriveToNote(Drive drive, String notePosChar) {
    return Commands.run(
        () -> {
          double speed = 0;
          if (notePosChar == "Center") {
            speed = 0.5;
          }
          drive.runVelocity(
              ChassisSpeeds.fromRobotRelativeSpeeds(0, speed, 0, drive.getRotation()));
        });
  }
}
