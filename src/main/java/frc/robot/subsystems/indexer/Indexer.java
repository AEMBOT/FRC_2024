package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexerIO.IndexerIOInputs.MotorState.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  }

  public Command getDefaultCommand(BooleanSupplier pivotHandoff) {
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
                  indexOffIntakeBack();
                }));
  }

  public void indexOffIntakeOn() {
    io.setShooterIndexer(OFF);
    io.setIntakeIndexer(IN);
  }

  public void indexOnIntakeOn() {
    io.setShooterIndexer(IN);
    io.setIntakeIndexer(IN);
  }

  public void indexOffIntakeOff() {
    io.setIntakeIndexer(OFF);
    io.setShooterIndexer(OFF);
  }

  public void indexOffIntakeBack() {
    io.setIntakeIndexer(OUT);
    io.setShooterIndexer(OFF);
  }

  public void indexerIn() {
    io.setShooterIndexer(IN);
  }

  public void indexerOut() {
    io.setShooterIndexer(OUT);
  }

  public void indexerStop() {
    io.setShooterIndexer(OFF);
  }

  public void intakeIn() {
    io.setIntakeIndexer(IN);
  }

  public void intakeOut() {
    io.setIntakeIndexer(OUT);
  }

  public void intakeStop() {
    io.setIntakeIndexer(OFF);
  }

  public void setShootReady(boolean value) {
    shootReady = value;
  }

  // Commands

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
