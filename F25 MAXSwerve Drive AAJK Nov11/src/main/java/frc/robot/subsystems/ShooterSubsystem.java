package frc.robot.subsystems;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.RelativeEncoder;

public class ShooterSubsystem extends SubsystemBase {
  private final SparkFlex m_leftMotor;
  private final SparkFlex m_rightMotor;
  private final SparkClosedLoopController m_leftController;
  private final SparkClosedLoopController m_rightController;
  private final RelativeEncoder m_leftEncoder;
  private final RelativeEncoder m_rightEncoder;
  
  private static final int CURRENT_LIMIT_AMPS = 80;
  private static final int LEFT_CAN_ID = 20;
  private static final int RIGHT_CAN_ID = 21;
  private static final double RPM_TARGET = 2000.0;
  private static final double TOLERANCE_RPM = 100.0;

  // PID Constants - TUNE THESE VALUES
  private static final double kP = 0.0001;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kFF = 0.000175;

  public ShooterSubsystem() {
    m_leftMotor = new SparkFlex(LEFT_CAN_ID, MotorType.kBrushless);
    m_rightMotor = new SparkFlex(RIGHT_CAN_ID, MotorType.kBrushless);
    m_leftController = m_leftMotor.getClosedLoopController();
    m_rightController = m_rightMotor.getClosedLoopController();
    m_leftEncoder = m_leftMotor.getEncoder();
    m_rightEncoder = m_rightMotor.getEncoder();

    SparkFlexConfig config = new SparkFlexConfig();
    
    // Configure current limit
    config.smartCurrentLimit(CURRENT_LIMIT_AMPS);
    
    // Configure encoder
    config.encoder
        .positionConversionFactor(1.0) 
        .velocityConversionFactor(1.0);

    // Configure PID
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(kP, kI, kD)
        .velocityFF(kFF);
    // Apply configuration
    m_leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    m_rightMotor.setInverted(true);
    
    // Set right motor to follow left (optional - if they should run together)
    // m_rightMotor.follow(m_leftMotor, true); // true if inverted
  }

  public void setShooterSpeed(double rpm) {
    m_leftController.setReference(rpm, ControlType.kVelocity);
    m_rightController.setReference(rpm, ControlType.kVelocity);
  }

  public void stop() {
    m_leftMotor.stopMotor();
    m_rightMotor.stopMotor();
  }

  public boolean atSpeed() {
    return Math.abs(m_leftEncoder.getVelocity() - RPM_TARGET) < TOLERANCE_RPM &&
           Math.abs(m_rightEncoder.getVelocity() - RPM_TARGET) < TOLERANCE_RPM;
  }

  public double getLeftVelocity() {
    return m_leftEncoder.getVelocity();
  }

  public double getRightVelocity() {
    return m_rightEncoder.getVelocity();
  }
  /** Spin the shooter motors to the target RPM. */
  public void spinUp(double rpm) {
    SmartDashboard.putNumber("Shooter Target RPM", rpm);
    m_leftController.setReference(rpm, com.revrobotics.spark.SparkBase.ControlType.kVelocity, com.revrobotics.spark.ClosedLoopSlot.kSlot0);
    m_rightController.setReference(rpm, com.revrobotics.spark.SparkBase.ControlType.kVelocity, com.revrobotics.spark.ClosedLoopSlot.kSlot0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Left RPM", getLeftVelocity());
    SmartDashboard.putNumber("Shooter Right RPM", getRightVelocity());
    SmartDashboard.putBoolean("Shooter At Speed", atSpeed());
  }
}