package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

public class IntakeSubsytem extends SubsystemBase {
    // Constants
    private static final int CURRENT_LIMIT_AMPS = 40;
    private static final double MAX_VELOCITY = 5000.0; // RPM
    private static final double MAX_ACCELERATION = 10000.0; // RPM/s
    private static final double GEAR_RATIO = 1.0; // Adjust if necessary
    private static final double ERR_ALLOW = 0.5; // Allowed error in RPM

    // Motor Controller
    private final SparkFlex intakeMotor = new SparkFlex(24, MotorType.kBrushless);

    // Encoder and PID Controller
    private final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
    private final SparkClosedLoopController m_pidController;

    // PID Tuning Variables
    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

    public IntakeSubsytem() {
        // 1. Initial PID Coefficients and Limits
        kP = 0.01;
        kI = 0.0;
        kD = 0.001;
        kIz = 0.0;
        kFF = 0.0002;
        kMaxOutput = 1.0;
        kMinOutput = -1.0;

        // 2. Instantiate PID Controller
        m_pidController = intakeMotor.getClosedLoopController();

        // 3. Configure Motor using SparkMaxConfig
        SparkFlexConfig intakeConfig = new SparkFlexConfig();

        // --- General Motor Parameters ---
        intakeConfig.idleMode(IdleMode.kBrake);
        intakeConfig.smartCurrentLimit(CURRENT_LIMIT_AMPS);

        // --- Encoder Conversion Factor ---
        intakeConfig.encoder.positionConversionFactor(1.0 / GEAR_RATIO); // Adjust if necessary
        intakeConfig.encoder.velocityConversionFactor(1.0 / GEAR_RATIO);
        intakeEncoder.setPosition(0);

        // --- Closed Loop Configuration ---
        intakeConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(kP)
            .i(kI)
            .d(kD)
            .velocityFF(kFF)
            .iZone(kIz)
            .outputRange(kMinOutput, kMaxOutput);

        intakeConfig.closedLoop.maxMotion
            .maxVelocity(MAX_VELOCITY)
            .maxAcceleration(MAX_ACCELERATION)
            .allowedClosedLoopError(ERR_ALLOW);

        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

        // 4. Display Tuning Variables on SmartDashboard
        SmartDashboard.putNumber("Intake P Gain", kP);
        SmartDashboard.putNumber("Intake I Gain", kI);
        SmartDashboard.putNumber("Intake D Gain", kD);
        SmartDashboard.putNumber("Intake I Zone", kIz);
        SmartDashboard.putNumber("Intake FF", kFF);
        SmartDashboard.putNumber("Intake Max Output", kMaxOutput);
        SmartDashboard.putNumber("Intake Min Output", kMinOutput);
        SmartDashboard.putNumber("Intake Target RPM", 0);
    }

    @Override
    public void periodic() {
        // 1. Read and update PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("Intake P Gain", kP);
        double i = SmartDashboard.getNumber("Intake I Gain", kI);
        double d = SmartDashboard.getNumber("Intake D Gain", kD);
        double iz = SmartDashboard.getNumber("Intake I Zone", kIz);
        double ff = SmartDashboard.getNumber("Intake FF", kFF);
        double max = SmartDashboard.getNumber("Intake Max Output", kMaxOutput);
        double min = SmartDashboard.getNumber("Intake Min Output", kMinOutput);

        // If PID coefficients have changed, update them
        if (p != kP || i != kI || d != kD || iz != kIz || ff != kFF || max != kMaxOutput || min != kMinOutput) {
            SparkFlexConfig updatedConfig = new SparkFlexConfig();
            updatedConfig.closedLoop
                .p(p)
                .i(i)
                .d(d)
                .velocityFF(ff)
                .iZone(iz)
                .outputRange(min, max);
            
            intakeMotor.configure(updatedConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

            kP = p; kI = i; kD = d; kIz = iz; kFF = ff;
            kMaxOutput = max; kMinOutput = min;
        }

        // 2. Display Telemetry
        SmartDashboard.putNumber("Intake RPM", intakeEncoder.getVelocity());
    }

    /**
     * Gets the maximum RPM configured for this intake.
     * @return The maximum RPM value.
     */
    public double getMaxRPM() {
        return MAX_VELOCITY;
    }

    /**
     * Sets the desired target RPM for the intake motor.
     * @param targetRPM The desired RPM.
     */
    public void setRPM(double targetRPM) {
        m_pidController.setReference(
                targetRPM,
                ControlType.kVelocity,
                ClosedLoopSlot.kSlot0
        );

        SmartDashboard.putNumber("Intake Target RPM", targetRPM);
    }

    public void stop() {
        intakeMotor.stopMotor();
        SmartDashboard.putNumber("Intake Target RPM", 0);
    }
}
