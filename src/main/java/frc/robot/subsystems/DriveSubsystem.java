package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SerialPort;

import com.revrobotics.RelativeEncoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  // The motors on drive system
  public static CANSparkMax m_frontLeftMotor = new CANSparkMax(DriveConstants.kFrontLeftWheelPort, MotorType.kBrushless);
  public static CANSparkMax m_frontRightMotor = new CANSparkMax(DriveConstants.kFrontRightWheelPort, MotorType.kBrushless);
  public static CANSparkMax m_backLeftMotor = new CANSparkMax(DriveConstants.kBackLeftWheelPort, MotorType.kBrushless);
  public static CANSparkMax m_backRightMotor = new CANSparkMax(DriveConstants.kBackRightWheelPort, MotorType.kBrushless);
  private static RelativeEncoder m_frontLeftEncoder = m_frontLeftMotor.getEncoder();
  private static RelativeEncoder m_frontRightEncoder = m_frontRightMotor.getEncoder();
  private static RelativeEncoder m_backLeftEncoder = m_backLeftMotor.getEncoder();
  private static RelativeEncoder m_backRightEncoder = m_backRightMotor.getEncoder();
 
  private AHRS navx = new AHRS(SerialPort.Port.kMXP);

  private final Field2d m_field = new Field2d();

  private final MecanumDriveKinematics m_kinematics = DriveConstants.kDriveKinematics;
   // Odometry class for tracking robot pose
  private final MecanumDriveOdometry m_odometry = new MecanumDriveOdometry(m_kinematics, navx.getRotation2d());

  private MecanumDrive m_drive = new MecanumDrive(m_frontLeftMotor, m_backLeftMotor, m_frontRightMotor,
      m_backRightMotor);
  
  NetworkTableEntry m_xEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("X");
  NetworkTableEntry m_yEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Y");
  NetworkTableEntry m_headingEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Heading");
  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {

    m_drive.setSafetyEnabled(false);
    m_frontLeftMotor.setInverted(false);
    m_frontRightMotor.setInverted(true);
    m_backLeftMotor.setInverted(false);
    m_backRightMotor.setInverted(true);

    motorBrake();

    m_odometry.resetPosition(new Pose2d(0,0, new Rotation2d(0)), new Rotation2d(0));

    setEncoders();
    setEncoderVelo();
    SmartDashboard.putData("Field", m_field);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(navx.getRotation2d(), getCurrentWheelSpeeds());
    m_field.setRobotPose(getPose());

    var translation = getPose().getTranslation();
    m_xEntry.setNumber(translation.getX());
    m_yEntry.setNumber(translation.getY());
    m_headingEntry.setNumber(m_odometry.getPoseMeters().getRotation().getDegrees());
  }

  public void setEncoders() {
    m_frontLeftEncoder.setPositionConversionFactor(DriveConstants.encoderConversionFactor);
    m_frontRightEncoder.setPositionConversionFactor(DriveConstants.encoderConversionFactor);
    m_backLeftEncoder.setPositionConversionFactor(DriveConstants.encoderConversionFactor);
    m_backRightEncoder.setPositionConversionFactor(DriveConstants.encoderConversionFactor);
  }

  public void setEncoderVelo() {
    m_frontLeftEncoder.setVelocityConversionFactor(DriveConstants.encoderVeloConversionFactor);
    m_frontRightEncoder.setVelocityConversionFactor(DriveConstants.encoderVeloConversionFactor);
    m_backLeftEncoder.setVelocityConversionFactor(DriveConstants.encoderVeloConversionFactor);
    m_backRightEncoder.setVelocityConversionFactor(DriveConstants.encoderVeloConversionFactor);
  }

  public void motorBrake(){
    m_frontLeftMotor.setIdleMode(IdleMode.kBrake);
    m_frontRightMotor.setIdleMode(IdleMode.kBrake);
    m_backLeftMotor.setIdleMode(IdleMode.kBrake);
    m_backRightMotor.setIdleMode(IdleMode.kBrake);
  }

  public void motorCoast(){
    m_frontLeftMotor.setIdleMode(IdleMode.kCoast);
    m_frontRightMotor.setIdleMode(IdleMode.kCoast);
    m_backLeftMotor.setIdleMode(IdleMode.kCoast);
    m_backRightMotor.setIdleMode(IdleMode.kCoast);
  }
  /**
   * Drives the robot using base mecanum (y stick 1 = forward, x stick 1 = sideways, x stick 2 = rotation)
   *
   * @param x = speed in x direction
   * @param y = speed in y direction
   * @param c = rotation speed
   */
  public void mecanumDrive(double x, double y, double c) {
    m_drive.driveCartesian(x,-y,c);
  }
  /**
   * Drives the robot using base mecanum (y stick 1 = forward, x stick 1 = sideways, x stick 2 = rotation)
   * This, however, is in relation to the field instead of the robot
   * @param x = speed in x direction
   * @param y = speed in y direction
   * @param c = rotation speed
   * @param theta = navx gyro angle
   */
  public void mecanumDriveGyro(double x, double y, double c, double theta)
  {
      m_drive.driveCartesian(-x, y, c, theta);
  }
  

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(m_frontLeftEncoder.getVelocity(),
      m_frontRightEncoder.getVelocity(),
      m_backLeftEncoder.getVelocity(),
      m_backRightEncoder.getVelocity());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, navx.getRotation2d());
  }
  
  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void setDriveMotorControllersVolts(MecanumDriveMotorVoltages volts){
    m_frontLeftMotor.setVoltage(volts.frontLeftVoltage);
    m_frontRightMotor.setVoltage(volts.frontRightVoltage);
    m_backLeftMotor.setVoltage(volts.rearLeftVoltage);
    m_backRightMotor.setVoltage(volts.rearRightVoltage);
}

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    m_frontLeftEncoder.setPosition(0);
    m_frontRightEncoder.setPosition(0);
    m_backLeftEncoder.setPosition(0);
    m_backRightEncoder.setPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_frontLeftEncoder.getPosition() + m_frontRightEncoder.getPosition()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public RelativeEncoder getFrontLeftEncoder() {
    return m_frontLeftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public RelativeEncoder getFrontRightEncoder() {
    return m_frontRightEncoder;
  }
  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public RelativeEncoder getBackLeftEncoder() {
    return m_backLeftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public RelativeEncoder getBackRightEncoder() {
    return m_backRightEncoder;
  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    navx.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return navx.getAngle();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -navx.getRate();
  }

  public void speedLimit(double limit){
    DriveConstants.speed = limit;
  }

  public void resetNavx() {
    navx.reset();
  }
}
