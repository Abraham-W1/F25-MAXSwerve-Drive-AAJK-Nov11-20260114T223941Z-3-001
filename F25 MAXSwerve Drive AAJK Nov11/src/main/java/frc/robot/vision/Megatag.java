package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.DriveSubsystem; // Changed from SwerveSubsystem
import java.util.ArrayList;
import java.util.List;

public class Megatag {
  private static final List<LimelightConfig> limelights = new ArrayList<>();
  private static DriveSubsystem driveSubsystem; // Changed to non-static

  // Add this method to set the drive subsystem instance
  public static void setDriveSubsystem(DriveSubsystem drive) {
    driveSubsystem = drive;
  }

    // Add this method to register Limelight cameras
    public static void addLimelight(LimelightConfig limelight) {
      limelights.add(limelight);
    }
  // Using Megatag 1

  public static void updateOdometry(LimelightConfig limelight) {
    LimelightHelpers.setPipelineIndex(limelight.name(), 1); // Use AprilTag pipeline
    boolean doRejectUpdate = false;

    LimelightHelpers.PoseEstimate mt1 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight.name());

    if (mt1 == null) {
      return;
    }

    Matrix<N3, N1> confid = getStdDev(mt1);

    if (mt1.tagCount == 0 || mt1.rawFiducials.length == 0) {
      doRejectUpdate = true;
    }

    if (!doRejectUpdate) {
      if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
        if (mt1.rawFiducials[0].ambiguity > limelight.ambiguity()) {
          doRejectUpdate = true;
        }
        if (mt1.rawFiducials[0].distToCamera > limelight.distToCamera()) {
          doRejectUpdate = true;
        }
      } else {
        doRejectUpdate = true;
      }
    }

    if (!doRejectUpdate) {
      driveSubsystem.addVisionMeasurement(mt1.pose, mt1.timestampSeconds, confid); // Changed reference
    }
  }

  public static Matrix<N3, N1> getStdDev(LimelightHelpers.PoseEstimate mt1pos) {

    double xyStdev = 2.0;

    double thetaStdev = 2.0;

    if (mt1pos.tagCount >= 2 && mt1pos.avgTagArea > 0.1) {
      xyStdev = 0.5;
      thetaStdev = 0.5;
    } else if (mt1pos.tagCount >= 2 || mt1pos.avgTagArea > 0.1) {
      xyStdev = 0.75;
      thetaStdev = 0.75;
    } else if (mt1pos.avgTagArea > 0.8) {
      xyStdev = 0.8;
      thetaStdev = 0.8;
    }
    Matrix<N3, N1> confidenceStdDev = VecBuilder.fill(xyStdev, xyStdev, thetaStdev);
    return confidenceStdDev;
  }

  public static void updateAllOdometry() {
    limelights.forEach((limelight) -> updateOdometry(limelight));
  }

  public static void updateIMU(LimelightConfig limelight, int mode) {
    LimelightHelpers.SetIMUMode(limelight.name(), mode);
  }

  public static void updateAllIMU(int mode) {
    limelights.forEach((limelight) -> updateIMU(limelight, mode));
  }

  public record LimelightConfig(String name, double ambiguity, double distToCamera) {}
}
