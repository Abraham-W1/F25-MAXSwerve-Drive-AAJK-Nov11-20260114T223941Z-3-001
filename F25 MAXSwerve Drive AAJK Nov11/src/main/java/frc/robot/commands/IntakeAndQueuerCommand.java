package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsytem;
import frc.robot.subsystems.QueuerSubsytem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Command to run both intake and queuer simultaneously.
 */
public class IntakeAndQueuerCommand extends Command {
  private final IntakeSubsytem intake;
  private final QueuerSubsytem queuer;
  private final double intakeSpeed;
  private final double queuerSpeed;

  public IntakeAndQueuerCommand(IntakeSubsytem intake, QueuerSubsytem queuer, 
                                 double intakeSpeed, double queuerSpeed) {
    this.intake = intake;
    this.queuer = queuer;
    this.intakeSpeed = intakeSpeed;
    this.queuerSpeed = queuerSpeed;
    
    // Add BOTH subsystems as requirements
    addRequirements(intake, queuer);
  }

  @Override
  public void execute() {
    intake.setRPM(intake.getMaxRPM() * intakeSpeed);
    queuer.setRPM(queuer.getMaxRPM() * queuerSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
    queuer.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}