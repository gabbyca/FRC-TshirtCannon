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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.commands.AlignToGoal;
import frc.robot.commands.AutoShooterCommand;
import frc.robot.commands.ConveyorCommand;
import frc.robot.commands.DropIntake;
import frc.robot.commands.FeederCommand;
import frc.robot.commands.FieldOrientedDrive;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LiftCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.SimpleAuto;
import frc.robot.commands.SpeedControl;
import frc.robot.commands.TurretCommand;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  Joystick m_joystick1 = new Joystick(JoystickConstants.kJoystick1Port);
  Joystick m_joystick2 = new Joystick(JoystickConstants.kJoystick2Port);
  
  //subsystems
  final DriveSubsystem m_drive = new DriveSubsystem();
  final IntakeSubsystem m_intake = new IntakeSubsystem();
  final ShooterSubsystem m_shooter = new ShooterSubsystem();
  final LiftSubsystem m_lift = new LiftSubsystem();
  final ConveyorSubsystem m_conveyor = new ConveyorSubsystem();
  final FeederSubsystem m_feeder = new FeederSubsystem();
  final TurretSubsystem m_turret = new TurretSubsystem();
  final BlinkinSubsystem m_led = new BlinkinSubsystem();
  final LimelightSubsystem m_camera = new LimelightSubsystem();
  //commands
  private final FieldOrientedDrive m_FOD = new FieldOrientedDrive(m_drive, () -> m_joystick1.rightY(),
   () -> m_joystick1.leftX(),
    () -> m_joystick1.rightX());
  
  private final AlignToGoal m_alignToGoal = new AlignToGoal(m_drive, m_camera);

  private final IntakeCommand m_runIntake = new IntakeCommand(m_intake, IntakeConstants.kIntakeSpeed);
  private final IntakeCommand m_stopIntake = new IntakeCommand(m_intake, 0);

  private final IntakeCommand m_backIntake = new IntakeCommand(m_intake, -IntakeConstants.kIntakeSpeed);

  private final ShooterCommand m_runShooter = new ShooterCommand(m_shooter, m_camera, ShooterConstants.shooterSpeed);
  private final ShooterCommand m_stopShooter = new ShooterCommand(m_shooter, m_camera, 0);

  private final AutoShooterCommand m_autoShooter = new AutoShooterCommand(m_shooter, m_conveyor, m_feeder, m_led, m_camera, ShooterConstants.shooterSpeed);

  private final LiftCommand m_liftUp = new LiftCommand(m_lift, m_led, LiftConstants.topPose);
  private final LiftCommand m_liftDown = new LiftCommand(m_lift, m_led, LiftConstants.bottomPose);

  private final TurretCommand m_turretLeft = new TurretCommand(m_turret, TurretConstants.kTurretSpeed);
  private final TurretCommand m_turretRight = new TurretCommand(m_turret, -TurretConstants.kTurretSpeed);
  private final TurretCommand m_turretStop = new TurretCommand(m_turret, 0);

  private final ConveyorCommand m_runConveyor = new ConveyorCommand(m_conveyor, ConveyorConstants.kConveyorSpeed);
  private final ConveyorCommand m_stopConveyor = new ConveyorCommand(m_conveyor, 0);

  private final ConveyorCommand m_backConveyor = new ConveyorCommand(m_conveyor, -ConveyorConstants.kConveyorSpeed);

  private final FeederCommand m_runFeeder = new FeederCommand(m_feeder, FeederConstants.kFeederSpeed);
  private final FeederCommand m_stopFeeder = new FeederCommand(m_feeder, 0);

  private final FeederCommand m_backFeeder = new FeederCommand(m_feeder, -FeederConstants.kFeederSpeed);

  private final DropIntake m_raiseIntake = new DropIntake(m_intake, -IntakeConstants.kIntakeDropSpeed, IntakeConstants.kIntakeDropTime);
  private final DropIntake m_dropIntake = new DropIntake(m_intake, IntakeConstants.kIntakeDropSpeed, IntakeConstants.kIntakeDropTime);
  private final DropIntake m_holdIntake = new DropIntake(m_intake, 0, 0);

  private final SpeedControl m_slowMode = new SpeedControl(m_led, 0.5);
  private final SpeedControl m_fastMode = new SpeedControl(m_led,1);

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

    m_joystick1.a()
        .whileHeld(m_backIntake)
        .whenReleased(m_stopIntake)
        .whileHeld(m_backConveyor)
        .whenReleased(m_stopConveyor)
        .whileHeld(m_backFeeder)
        .whenReleased(m_stopFeeder);   
    
    m_joystick1.x()
      .whenPressed(m_runShooter);

    m_joystick1.b()
      .whenPressed(m_stopShooter);
    
    m_joystick1.y()
      .whenPressed(m_autoShooter); 

    m_joystick1.leftBumper()
      .whileHeld(m_fastMode)
      .whenReleased(m_slowMode);

    m_joystick1.rightBumper()
      .whenPressed(m_runIntake)
        .whenReleased(m_stopIntake)
      .whileHeld(m_runConveyor)
        .whenReleased(m_stopConveyor);

    m_joystick1.back()
      .whenPressed(m_dropIntake)
      .whenReleased(m_holdIntake);

    m_joystick1.start()
      .whenPressed(m_raiseIntake)
      .whenReleased(m_holdIntake);
      
    m_joystick1.dpadUp()
      .whenPressed(m_liftUp);
    
    m_joystick1.dpadDown()
      .whenPressed(m_liftDown);

    m_joystick1.dpadRight()
      .whenPressed(m_turretRight)
      .whenReleased(m_turretStop);
    
    m_joystick1.dpadLeft()
      .whenPressed(m_turretLeft)
      .whenReleased(m_turretStop);

  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  void configureTestBindings() {

    m_joystick1.a()
      .whenPressed(m_runIntake)
      .whenReleased(m_stopIntake);
    
    m_joystick1.x()
      .whenPressed(m_runShooter);

    m_joystick1.b()
      .whenPressed(m_stopShooter);
    
    m_joystick1.y()
      .whenPressed(m_alignToGoal); 

    m_joystick1.leftBumper()
      .whileHeld(m_fastMode)
      .whenReleased(m_slowMode);

    m_joystick1.rightBumper()
      .whileHeld(m_runConveyor)
      .whenReleased(m_stopConveyor);

    m_joystick1.back()
      .whileHeld(m_runFeeder)
      .whenReleased(m_stopFeeder);

    m_joystick1.start()
      .whenPressed(m_dropIntake)
      .whenReleased(m_holdIntake);

    m_joystick1.leftStick()
      .whenPressed(m_raiseIntake)
      .whenReleased(m_holdIntake);

    m_joystick1.dpadUp()
      .whenPressed(m_liftUp);
    
    m_joystick1.dpadDown()
      .whenPressed(m_liftDown);

    m_joystick1.dpadRight()
      .whenPressed(m_turretRight)
      .whenReleased(m_turretStop);
    
    m_joystick1.dpadLeft()
      .whenPressed(m_turretLeft)
      .whenReleased(m_turretStop);

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
    return null;
    //mecanumControllerCommand.andThen(() -> m_drive.mecanumDrive(0, 0, 0));
  }
}
