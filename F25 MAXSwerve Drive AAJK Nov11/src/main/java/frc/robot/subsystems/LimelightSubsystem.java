package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.vision.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase {
    private final String limelightName = "limelight-shooter"; //default

    public LimelightSubsystem(){
        LimelightHelpers.setLEDMode_ForceOn(limelightName);  // Force LEDs ON
        LimelightHelpers.setPipelineIndex(limelightName, 0);
    }

    public boolean hasTarget(){
        return LimelightHelpers.getTV(limelightName);
    }
    public double getTargetX(){
        return LimelightHelpers.getTX(limelightName);
    }
    public double getTargetY(){
        return LimelightHelpers.getTY(limelightName);
    }
    public double getTargetA(){
        return LimelightHelpers.getTA(limelightName);
    }
    public void setPipeline(int pipeline){
        LimelightHelpers.setPipelineIndex(limelightName, pipeline);
    }

    //Turning on and off LEDs to be controlled from command
    public void enableLED(){
        LimelightHelpers.setLEDMode_ForceOn(limelightName);
    }
    public void disableLED(){
        LimelightHelpers.setLEDMode_ForceOff(limelightName);
    }
    //Prints values to smart dashboard
    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Limelight has target: ", hasTarget());
        SmartDashboard.putNumber("Limelight TX: ", getTargetX());
        SmartDashboard.putNumber("Limelight TY: ", getTargetY());
        SmartDashboard.putNumber("Limelight TA: ", getTargetA());
    }
}