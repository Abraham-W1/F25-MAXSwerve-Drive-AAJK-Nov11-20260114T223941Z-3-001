package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.QueuerSubsytem;

public class SpinUpShooterCommand extends Command {
  private final ShooterSubsystem shooter;
  private final double targetRPM;

  public SpinUpShooterCommand(ShooterSubsystem shooter, double rpm) {
    this.shooter = shooter;
    this.targetRPM = rpm;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.spinUp(targetRPM);
  }

  @Override
  public void execute() {
    // Nothing to do; spinUp just maintains
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  @Override
  public boolean isFinished() {
    return false; // runs until interrupted
  }
}
