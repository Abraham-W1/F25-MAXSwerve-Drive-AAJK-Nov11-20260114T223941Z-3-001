package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.vision.TagAlignment; // Changed from Megatag
import frc.robot.vision.LimelightHelpers;

public class AlignToTagCommand extends Command {
  private final DriveSubsystem drive;
  private final String limelightName;
  private final double kP = 0.035;
  
  public AlignToTagCommand(DriveSubsystem drive, String limelightName) {
    this.drive = drive;
    this.limelightName = limelightName;
    addRequirements(drive);
        System.out.println("Empty File");
        System.out.println("More emptiness");
  }

  @Override
  public void execute() {
    double rotSpeed = TagAlignment.aimAtTag(limelightName, kP);
    drive.drive(0, 0, rotSpeed, true);
  }

  @Override
  public boolean isFinished() {
    if (!LimelightHelpers.getTV(limelightName)) {
      return false;
    }

    return Math.abs(LimelightHelpers.getTX(limelightName)) < 2.0;

  }


  @Override
  public void end(boolean interrupted) {
    drive.drive(0, 0, 0, true);
  }
}
