package frc.robot.vision;

public class TagAlignment {
  
  /**
   * Calculates rotational velocity to aim at an AprilTag
   * @param limelightName Name of the Limelight to use
   * @param kP Proportional gain (start with 0.035)
   * @return Angular velocity to turn toward target
   */
  public static double aimAtTag(String limelightName, double kP) {
    if (!LimelightHelpers.getTV(limelightName)) {
      return 0.0; // No target, don't rotate
    }

    double tx = LimelightHelpers.getTX(limelightName);
    double targetingAngularVelocity = tx * kP;
    return -targetingAngularVelocity; // Negative to turn toward target
  }

  /**
   * Check if robot is aligned with tag (within tolerance)
   * @param limelightName Name of the Limelight
   * @param toleranceDegrees Alignment tolerance (default: 2.0)
   * @return true if aligned
   */
  public static boolean isAlignedWithTag(String limelightName, double toleranceDegrees) {
    if (!LimelightHelpers.getTV(limelightName)) {
      return false;
    }
    return Math.abs(LimelightHelpers.getTX(limelightName)) < toleranceDegrees;
  }

  /**
   * Check if a valid AprilTag target is visible
   * @param limelightName Name of the Limelight
   * @return true if target detected
   */
  public static boolean hasValidTarget(String limelightName) {
    return LimelightHelpers.getTV(limelightName);
  }
}
