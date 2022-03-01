package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private static CANSparkMax m_intake = new CANSparkMax(IntakeConstants.kIntakePort, MotorType.kBrushed);

    private  static CANSparkMax m_intakeDrop = new CANSparkMax(IntakeConstants.kIntakeDropPort, MotorType.kBrushed);

    private final AnalogInput m_ultrasonic = new AnalogInput(0);

    private static int m_count;
    /**
     * this is the class for the intake
     * thats it its really simple
     */
    public IntakeSubsystem() {
        m_intake.setIdleMode(IdleMode.kCoast);
        m_intakeDrop.setIdleMode(IdleMode.kBrake);
        m_count = 0;
    }
    
    /**
     * will i keep the name as succ? probably can you change it? no.
     * 
     * @param speed = speed at which succage occurs
     * @throws InterruptedException
     */
    public void succ(double speed){
            m_intake.set(speed);
    }
    
    public void drop(double speed){
            m_intakeDrop.set(speed);
    }
    public int getCount(){
        return m_count;
    }

    public void gotOne(){
        m_count++;
    }

    public void shotOne(){
        m_count--;
    }

    public boolean inIntake(){
        return getInches() < 12;
    }

    public double getInches(){
        double voltage_scale_factor = 5/RobotController.getVoltage5V();
        double rawValue = m_ultrasonic.getValue();
        return rawValue * voltage_scale_factor * 0.0492;
    }

    public double getMM(){
        double voltage_scale_factor = 5/RobotController.getVoltage5V();
        double rawValue = m_ultrasonic.getValue();
        return rawValue * voltage_scale_factor * 0.125;
    }
}