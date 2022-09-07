package frc.robot.subsystems; 

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import io.github.oblarg.oblog.Logger;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;


public class DifferentialDriveSubsystem {



    public static CANSparkMax m_leftMotor = new CANSparkMax(2,MotorType.kBrushless);
    public static CANSparkMax m_rightMotor = new CANSparkMax(6, MotorType.kBrushless);


    DifferentialDrive myRobot = new DifferentialDrive(m_leftMotor, m_rightMotor);

    public DifferentialDriveSubsystem(){

        //anything to be done when subsytem initiate 
        m_rightMotor.setInverted(false);
        m_leftMotor.setInverted(true);

    }

    public void drive(double x, double y){

        myRobot.tankDrive(x, y);

    }
    


}
