/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AlignToGoal;
import frc.robot.commands.FieldOrientedDrive;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.SpeedControl;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  Joystick m_joystick1 = new Joystick(JoystickConstants.kJoysick1Port);
  Joystick m_joystick2 = new Joystick(JoystickConstants.kJoystick2Port);
  
  //subsystems
  final DriveSubsystem m_drive = new DriveSubsystem();
  final IntakeSubsystem m_intake = new IntakeSubsystem();
  final ShooterSubsystem m_shooter = new ShooterSubsystem();
  final LimelightSubsystem m_camera = new LimelightSubsystem();
  //commands
  private final FieldOrientedDrive m_FOD = new FieldOrientedDrive(m_drive, () -> m_joystick1.getRawAxis(JoystickConstants.kYStick2),
   () -> m_joystick1.getRawAxis(JoystickConstants.kXStick1),
    () -> m_joystick1.getRawAxis(JoystickConstants.kXStick2));
  
  private final AlignToGoal m_alignToGoal = new AlignToGoal(m_drive, m_camera);
  private final IntakeCommand m_runIntake = new IntakeCommand(m_intake, IntakeConstants.kIntakeSpeed);
  private final IntakeCommand m_stopIntake = new IntakeCommand(m_intake, 0);
  private final ShooterCommand m_runShooter = new ShooterCommand(m_shooter, m_intake, m_camera, ShooterConstants.kIdealShotSpeed);
  private final ShooterCommand m_stopShooter = new ShooterCommand(m_shooter, m_intake, m_camera, 0);
  private final SpeedControl m_slowMode = new SpeedControl(0.5);
  private final SpeedControl m_fastMode = new SpeedControl(1);

  //chooser
  private SendableChooser<Command> autonomousChooser = new SendableChooser<Command>();
  private SendableChooser<Command> driveChooser = new SendableChooser<Command>();
  //limelight stuff
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ta = table.getEntry("ta");

  

  
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    //sets up our auto chooser(see the classes for details)
    //autonomousChooser.setDefaultOption("Trajectory", );
    //autonomousChooser.addOption("Shoot Left Of Target", a_autoDriveToLineAndShootLeft);
    //sets up drive chooser with the option between FOD and default
    driveChooser.setDefaultOption("Field Oriented Drive", m_FOD);
    //driveChooser.addOption("Default Drive", m_default);
    //publishes the choosers
    SmartDashboard.putData("Autonomous Mode", autonomousChooser);
    SmartDashboard.putData("Drive Mode", driveChooser);
    // Configure the button bindings
    configureButtonBindings();
    
    //sets the drive to what it should be
    m_drive.setDefaultCommand(driveChooser.getSelected());

    //guess
    SmartDashboard.putNumber("NAVX Angle", m_drive.getHeading());
    
    }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(m_joystick1, 1)
      .whenPressed(m_runIntake)
      .whenReleased(m_stopIntake);
    
    new JoystickButton(m_joystick1, 2)
      .whenPressed(m_runShooter);

    new JoystickButton(m_joystick1, 3)
      .whenPressed(m_stopShooter);
    
    new JoystickButton(m_joystick1, 4)
      .whenPressed(m_alignToGoal); 

    new JoystickButton(m_joystick1, 5)
      .whileHeld(m_fastMode)
      .whenReleased(m_slowMode);
    
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    MecanumControllerCommand mecanumControllerCommand =
        new MecanumControllerCommand(
            Trajectories.exampleTrajectory,
            m_drive::getPose,
            DriveConstants.kFeedforward,
            DriveConstants.kDriveKinematics,
            // Position contollers
            DriveConstants.xController,
            DriveConstants.yController,
            DriveConstants.thetaController,
            // Needed for normalizing wheel speeds
            DriveConstants.kMaxSpeedMetersPerSecond,
            // Velocity PID's
            DriveConstants.driveController,
            DriveConstants.driveController,
            DriveConstants.driveController,
            DriveConstants.driveController,
            m_drive::getCurrentWheelSpeeds,
            m_drive::setDriveMotorControllersVolts, // Consumer for the output motor voltages
            m_drive);
    // An ExampleCommand will run in autonomous
    return mecanumControllerCommand.andThen(() -> m_drive.mecanumDrive(0, 0, 0));
  }
}
