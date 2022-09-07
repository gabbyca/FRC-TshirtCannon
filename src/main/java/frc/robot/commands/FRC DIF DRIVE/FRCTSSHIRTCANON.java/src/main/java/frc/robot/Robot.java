/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DifferentialDriveSubsystem;
import io.github.oblarg.oblog.Logger;


import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private DifferentialDrive m_myRobot;
  //for turn

  DifferentialDriveSubsystem difDrive = new DifferentialDriveSubsystem(); 
  private Joystick m_rightStick = new Joystick(0); 
  private Joystick m_leftStick = new Joystick(0); 



  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {


    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    
    
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    Logger.updateEntries();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    // m_robotContainer.m_drive.motorCoast();
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // m_robotContainer.m_drive.motorBrake();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    // m_robotContainer.m_drive.motorBrake();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    //m_myRobot.tankDrive(m_leftStick.getY(), m_rightStick.getY());
    difDrive.drive((m_leftStick.leftY(), m_rightStick.rightX());
  
    
  

    // SmartDashboard.putNumber("NAVX Angle", m_robotContainer.m_drive.getHeading());
    // SmartDashboard.putNumber("Shooter Velocity", m_robotContainer.m_shooter.getVelocity());
    // SmartDashboard.putBoolean("Detect goal?", m_robotContainer.m_camera.hasTarget());
    // SmartDashboard.putNumber("Goal X", m_robotContainer.m_camera.getTX());
    // SmartDashboard.putNumber("Goal Y", m_robotContainer.m_camera.getTY());
    // SmartDashboard.putNumber("Lift Pose", m_robotContainer.m_lift.getPose());
    // SmartDashboard.putNumber("Turret Pose", m_robotContainer.m_turret.getPose());

    // m_robotContainer.m_shooter.shoot(ShooterConstants.shooterSpeed * m_robotContainer.m_camera.getShooterSpeed());

    // if (m_robotContainer.m_camera.alignedToGoal())
    //   m_robotContainer.m_led.violet();
    // else
    //   m_robotContainer.m_led.red();
  
    
  
    }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    
  }

  public static void wait(int ms)
    {
        try
        {
            Thread.sleep(ms); //core java delay command
        }
        catch(InterruptedException ex)
        {
            Thread.currentThread().interrupt(); //this exception is useful to remove the glitches and errors of the thread.sleep()
        }
    }

}
