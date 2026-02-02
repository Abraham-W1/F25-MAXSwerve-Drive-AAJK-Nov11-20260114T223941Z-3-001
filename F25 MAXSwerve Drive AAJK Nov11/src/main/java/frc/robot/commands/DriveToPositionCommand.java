package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToPositionCommand extends Command {
  private final DriveSubsystem drive;
  private final Pose2d targetPose;
  
  // PID controllers for x, y, and rotation
  private final PIDController xController = new PIDController(2.0, 0, 0);
  private final PIDController yController = new PIDController(2.0, 0, 0);
  private final PIDController rotController = new PIDController(3.0, 0, 0);
  
  private static final double POSITION_TOLERANCE = 0.05; // 5cm
  private static final double ANGLE_TOLERANCE = 2.0;     // 2 degrees
  
  /**
   * Drive to a specific field coordinate
   * @param drive The drive subsystem
   * @param x Target X coordinate (meters)
   * @param y Target Y coordinate (meters)
   * @param rotation Target rotation (degrees)
   */
  public DriveToPositionCommand(DriveSubsystem drive, double x, double y, double rotation) {
    this.drive = drive;
    this.targetPose = new Pose2d(x, y, Rotation2d.fromDegrees(rotation));
    
    // Enable continuous input for rotation (-180 to 180)
    rotController.enableContinuousInput(-180, 180);
    
    // Set tolerances
    xController.setTolerance(POSITION_TOLERANCE);
    yController.setTolerance(POSITION_TOLERANCE);
    rotController.setTolerance(ANGLE_TOLERANCE);
    
    addRequirements(drive);
  }
  
  @Override
  public void initialize() {
    // Reset PID controllers
    xController.reset();
    yController.reset();
    rotController.reset();
  }
  
  @Override
  public void execute() {
    Pose2d currentPose = drive.getPose();
    
    // Calculate velocities using PID
    double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
    double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
    double rotSpeed = rotController.calculate(
        currentPose.getRotation().getDegrees(),
        targetPose.getRotation().getDegrees()
    );
    
    // Limit speeds to max values (tune these)
    xSpeed = Math.max(-1.0, Math.min(1.0, xSpeed));
    ySpeed = Math.max(-1.0, Math.min(1.0, ySpeed));
    rotSpeed = Math.max(-1.0, Math.min(1.0, rotSpeed));
    
    // Drive with field-relative control
    drive.drive(xSpeed, ySpeed, rotSpeed, true);
  }
  
  @Override
  public boolean isFinished() {
    // Finished when at position and rotation
    return xController.atSetpoint() 
        && yController.atSetpoint() 
        && rotController.atSetpoint();
  }
  
  @Override
  public void end(boolean interrupted) {
    drive.drive(0, 0, 0, true); // Stop
  }
}
