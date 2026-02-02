package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AlignToTagCommand;
import frc.robot.commands.ArmCommands;
import frc.robot.commands.DriveToPositionCommand;
import frc.robot.commands.DriveToTag;
import frc.robot.commands.FeedShooterCommand;
import frc.robot.commands.IntakeAndQueuerCommand;
import frc.robot.commands.SpinUpShooterCommand;
import frc.robot.commands.FeedShooterCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsytem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.QueuerSubsytem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveToTag;
import java.util.List;
import frc.robot.vision.Megatag; // Add this import
import edu.wpi.first.math.MathUtil; // Likely missing for applyDeadband
import edu.wpi.first.wpilibj.XboxController; // Add if missing
import frc.robot.vision.LimelightConfigs;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ArmSubsystem arm = new ArmSubsystem();
  private final IntakeSubsytem intake = new IntakeSubsytem();
  private final QueuerSubsytem queuer = new QueuerSubsytem();
  private final LimelightSubsystem limelight = new LimelightSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    Megatag.setDriveSubsystem(m_robotDrive);
    Megatag.addLimelight(new Megatag.LimelightConfig("limelight", 0.5, 3.0));

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link PS4Controller}), and then passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // R1 button - X-stance (wheels in X formation to resist pushing)
    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(), 
            m_robotDrive));

    // L1 button - Control arm position
    new JoystickButton(m_driverController, Button.kLeftBumper.value)
        .whileTrue(new ArmCommands(arm, 45.0))
        .onFalse(new ArmCommands(arm, 0.0));

    // B button - Run intake and queuer together
    new JoystickButton(m_driverController, Button.kB.value)
        .whileTrue(new IntakeAndQueuerCommand(intake, queuer, -0.7, -0.5));

    //A Button - Drive to AprilTag using Limelight (auto aim & range)
    //new JoystickButton(m_driverController, Button.kA.value)
      //  .whileTrue(new DriveToTag(m_robotDrive, limelight));

    new JoystickButton(m_driverController, Button.kA.value) 
      .whileTrue(new AlignToTagCommand(m_robotDrive, "limelight"));

    //X Button - spin up shooter while held
    new JoystickButton(m_driverController, Button.kX.value)
        .whileTrue(new SpinUpShooterCommand(shooter, 2000)); // or whatever RPM

    //Y button - fire if shooter is at speed
    new JoystickButton(m_driverController, Button.kY.value)
        .onTrue(new FeedShooterCommand(shooter, queuer));

       //start button - fire if shooter is at speed
       new JoystickButton(m_driverController, Button.kStart.value)
       .onTrue(new InstantCommand(()->m_robotDrive.zeroHeading(),m_robotDrive) );    

      // D-Pad Up - Drive to Amp position (example coordinates)
  //new JoystickButton(m_driverController, Button.kUp.value)
    //  .onTrue(new DriveToPositionCommand(m_robotDrive, 1.84, 7.62, 90.0));
  
  // D-Pad Down - Drive to Source position
  //new JoystickButton(m_driverController, Button.kDown.value)
   //   .onTrue(new DriveToPositionCommand(m_robotDrive, 15.2, 1.0, 0.0));
  
  // D-Pad Left - Drive to Speaker scoring position
 // new JoystickButton(m_driverController, Button.kLeft.value)
//      .onTrue(new DriveToPositionCommand(m_robotDrive, 1.5, 5.5, -30.0));


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose,
        DriveConstants.kDriveKinematics,
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }
}