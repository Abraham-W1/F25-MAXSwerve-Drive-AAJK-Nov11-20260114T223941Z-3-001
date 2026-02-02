package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

public class QueuerSubsytem extends SubsystemBase {
    // Constants
    private static final int CURRENT_LIMIT_AMPS = 40;
    private static final double MAX_VELOCITY = 2000.0; // RPM (NEO Vortex free speed ~6700)
    private static final double GEAR_RATIO = 1.0; // Adjust based on your mechanism

    // Motor Controllers
    private final SparkFlex queuerMotor1 = new SparkFlex(22, MotorType.kBrushless);
    private final SparkFlex queuerMotor2 = new SparkFlex(23, MotorType.kBrushless);

    // Encoders and PID Controllers
    private final RelativeEncoder encoder1 = queuerMotor1.getEncoder();
    private final RelativeEncoder encoder2 = queuerMotor2.getEncoder();
    private final SparkClosedLoopController pidController1;
    private final SparkClosedLoopController pidController2;

    // PID Tuning Variables
    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

    public QueuerSubsytem() {
        // 1. Initial PID Coefficients
        kP = 0.001;
        kI = 0.0;
        kD = 0.001;
        kIz = 0.0;
        kFF = 0.0002;
        kMaxOutput = 1.0;
        kMinOutput = -1.0;

        // 2. Get PID Controllers
        pidController1 = queuerMotor1.getClosedLoopController();
        pidController2 = queuerMotor2.getClosedLoopController();

        // 3. Configure Both Motors
        SparkFlexConfig queuerConfig = new SparkFlexConfig();

        // General Parameters
        queuerConfig.idleMode(IdleMode.kBrake);
        queuerConfig.smartCurrentLimit(CURRENT_LIMIT_AMPS);

        // Encoder Conversion
        queuerConfig.encoder.positionConversionFactor(1.0 / GEAR_RATIO);
        queuerConfig.encoder.velocityConversionFactor(1.0 / GEAR_RATIO);

        // Closed Loop Configuration
        queuerConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(kP)
            .i(kI)
            .d(kD)
            .velocityFF(kFF)
            .iZone(kIz)
            .outputRange(kMinOutput, kMaxOutput);

        // Apply to both motors
        queuerMotor1.configure(queuerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        queuerMotor2.configure(queuerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Make motor 2 follow motor 1 (optional - removes need for dual PID calls)
        // queuerMotor2.follow(queuerMotor1, false); // false = same direction, true = inverted

        // Reset encoders
        encoder1.setPosition(0);
        encoder2.setPosition(0);

        // 4. SmartDashboard Tuning
        SmartDashboard.putNumber("Queuer P Gain", kP);
        SmartDashboard.putNumber("Queuer I Gain", kI);
        SmartDashboard.putNumber("Queuer D Gain", kD);
        SmartDashboard.putNumber("Queuer I Zone", kIz);
        SmartDashboard.putNumber("Queuer FF", kFF);
        SmartDashboard.putNumber("Queuer Max Output", kMaxOutput);
        SmartDashboard.putNumber("Queuer Min Output", kMinOutput);
        SmartDashboard.putNumber("Queuer Target RPM", 0);
    }

    @Override
    public void periodic() {
        // 1. Live PID Tuning
        double p = SmartDashboard.getNumber("Queuer P Gain", kP);
        double i = SmartDashboard.getNumber("Queuer I Gain", kI);
        double d = SmartDashboard.getNumber("Queuer D Gain", kD);
        double iz = SmartDashboard.getNumber("Queuer I Zone", kIz);
        double ff = SmartDashboard.getNumber("Queuer FF", kFF);
        double max = SmartDashboard.getNumber("Queuer Max Output", kMaxOutput);
        double min = SmartDashboard.getNumber("Queuer Min Output", kMinOutput);

        if (p != kP || i != kI || d != kD || iz != kIz || ff != kFF || max != kMaxOutput || min != kMinOutput) {
            SparkFlexConfig updatedConfig = new SparkFlexConfig();
            updatedConfig.closedLoop
                .p(p)
                .i(i)
                .d(d)
                .velocityFF(ff)
                .iZone(iz)
                .outputRange(min, max);
            
            queuerMotor1.configure(updatedConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            queuerMotor2.configure(updatedConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

            kP = p; kI = i; kD = d; kIz = iz; kFF = ff;
            kMaxOutput = max; kMinOutput = min;
        }

        // 2. Telemetry
        SmartDashboard.putNumber("Queuer Motor 1 RPM", encoder1.getVelocity());
        SmartDashboard.putNumber("Queuer Motor 2 RPM", encoder2.getVelocity());
    }

    /**
     * Sets both queuer motors to the same target RPM.
     * @param targetRPM Desired velocity in RPM
     */
    public void setRPM(double targetRPM) {
        pidController1.setReference(-targetRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        pidController2.setReference(targetRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        
        SmartDashboard.putNumber("Queuer Target RPM", targetRPM);
    }

    /**
     * Sets different RPMs for each motor (if needed for differential control).
     */
    public void setRPM(double motor1RPM, double motor2RPM) {
        pidController1.setReference(motor1RPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        pidController2.setReference(motor2RPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    /**
     * Stops both motors.
     */
    public void stop() {
        queuerMotor1.stopMotor();
        queuerMotor2.stopMotor();
        SmartDashboard.putNumber("Queuer Target RPM", 0);
    }

    /**
     * Gets the maximum RPM configured.
     */
    public double getMaxRPM() {
        return MAX_VELOCITY;
    }
}
