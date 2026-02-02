// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsytem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Command to run the intake at a specified percentage of maximum RPM.
 */
public class IntakeCommands extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsytem intake;
  private final double percentSpeed; // Percentage of max RPM (0.0 to 1.0)

  /**
   * Creates a new IntakeCommands.
   *
   * @param intake The intake subsystem used by this command.
   * @param percentSpeed The desired speed as a percentage of max RPM (0.0 to 1.0).
   *                     For example, 0.5 = 50% of max RPM, 1.0 = 100% of max RPM.
   *                     Can also be negative for reverse operation.
   */
  public IntakeCommands(IntakeSubsytem intake, double percentSpeed) {
    this.intake = intake;
    this.percentSpeed = percentSpeed;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Command will start running the intake at the specified speed
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Calculate target RPM as a percentage of MAX_RPM
    double targetRPM = intake.getMaxRPM() * percentSpeed;
    intake.setRPM(targetRPM);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the intake when command ends
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // Command runs until interrupted
  }
}