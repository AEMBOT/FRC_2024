package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  public boolean shootReady = false;

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
    Logger.recordOutput(
        "Indexer/Running Command",
        Optional.ofNullable(this.getCurrentCommand()).map(Command::getName).orElse("None"));
  }

  public Command getDefault(BooleanSupplier pivotHandoff) {
    return runOnce(() -> shootReady = false)
        .andThen(
            Commands.waitUntil(() -> inputs.shooterBeamBreakState)
                .deadlineWith(
                    run(this::indexOffIntakeOn)
                        .until(() -> inputs.intakeBeamBreakState)
                        .andThen(run(this::indexOffIntakeOff))
                        .until(pivotHandoff)
                        .andThen(run(this::indexOnIntakeOn))))
        .andThen(
            run(
                () -> {
                  shootReady = true;
                  indexOffIntakeOff();
                }))
        .finallyDo(this::indexOffIntakeOff);
  }

  public boolean intakedNote() {
    return inputs.intakeBeamBreakState;
  }

  public void indexOffIntakeOn() {
    io.setIndexerVoltage(0);
    io.setIntakeVoltage(IntakeConstants.intakeMotorVoltage);
  }

  public void indexOnIntakeOn() {
    io.setIndexerVoltage(IndexerConstants.indexerMotorVoltage);
    io.setIntakeVoltage(IntakeConstants.intakeMotorVoltage);
  }

  public void indexOffIntakeOff() {
    io.setIntakeVoltage(0);
    io.setIndexerVoltage(0);
  }

  public void indexOffIntakeBack() {
    io.setIntakeVoltage(-IntakeConstants.intakeMotorVoltage);
    io.setIndexerVoltage(0);
  }

  public void indexerIn() {
    io.setIndexerVoltage(IndexerConstants.indexerMotorVoltage);
  }

  public void indexerOut() {
    io.setIndexerVoltage(-IndexerConstants.indexerMotorVoltage);
  }

  public void indexerStop() {
    io.setIndexerVoltage(0);
  }

  public void intakeIn() {
    io.setIntakeVoltage(IntakeConstants.intakeMotorVoltage);
  }

  public void intakeOut() {
    io.setIntakeVoltage(-IntakeConstants.intakeMotorVoltage);
  }

  public void intakeStop() {
    io.setIntakeVoltage(0);
  }

  public boolean getShootReady() {
    return shootReady;
  }

  // Commands
  public Command shootCommand() {
    return run(
        () -> {
          io.setIndexerVoltage(8.0);
          io.setIntakeVoltage(0.0);
        });
  }

  public Command intakeInCommand() {
    return runOnce(this::intakeIn);
  }

  public Command intakeOutCommand() {
    return runOnce(this::intakeOut);
  }

  public Command intakeStopCommand() {
    return runOnce(this::intakeStop);
  }

  public Command indexerInCommand() {
    return runOnce(this::indexerIn);
  }

  public Command indexerOutCommand() {
    return runOnce(this::indexerOut);
  }

  public Command indexerStopCommand() {
    return runOnce(this::indexerStop);
  }
}
