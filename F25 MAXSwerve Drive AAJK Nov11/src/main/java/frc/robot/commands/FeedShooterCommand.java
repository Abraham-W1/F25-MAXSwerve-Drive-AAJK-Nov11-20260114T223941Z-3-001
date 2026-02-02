package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.QueuerSubsytem;

public class FeedShooterCommand extends Command {
  private final ShooterSubsystem shooter;
  private final QueuerSubsytem queuer;
  private static final double SHOOTER_READY_RPM = 2000.0;

  public FeedShooterCommand(ShooterSubsystem shooter, QueuerSubsytem queuer) {
    this.shooter = shooter;
    this.queuer = queuer;
    addRequirements(queuer);
  }

  @Override
  public void initialize() {
    // Command starts - prepare but don't feed yet
  }

  @Override
  public void execute() {
    // Continuously check shooter speed and feed if ready
    if (shooter.getRightVelocity() >= SHOOTER_READY_RPM) {
      queuer.setRPM(queuer.getMaxRPM() * 20.0); // Run queuer at 100% speed
    } else {
      queuer.stop(); // Stop if shooter slows down
    }
  }

  @Override
  public void end(boolean interrupted) {
    queuer.stop();
  }

  @Override
  public boolean isFinished() {
    return false; // Run until button released or interrupted
  }
}