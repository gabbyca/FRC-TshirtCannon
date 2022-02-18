package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private  CANSparkMax m_intake = new CANSparkMax(IntakeConstants.kIntakePort, MotorType.kBrushed);

    private final AnalogInput ultrasonic = new AnalogInput(0);

    private static int count;
    /**
     * this is the class for the intake
     * thats it its really simple
     */
    public IntakeSubsystem() {
        count = 0;
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
    
    public int getCount(){
        return count;
    }

    public void gotOne(){
        count++;
    }

    public void shotOne(){
        count--;
    }

    public boolean inIntake(){
        return getInches() < 12;
    }

    public double getInches(){
        double voltage_scale_factor = 5/RobotController.getVoltage5V();
        double rawValue = ultrasonic.getValue();
        return rawValue * voltage_scale_factor * 0.0492;
    }

    public double getMM(){
        double voltage_scale_factor = 5/RobotController.getVoltage5V();
        double rawValue = ultrasonic.getValue();
        return rawValue * voltage_scale_factor * 0.125;
    }
}