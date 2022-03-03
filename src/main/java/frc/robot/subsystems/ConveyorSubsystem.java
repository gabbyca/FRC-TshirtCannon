package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;

public class ConveyorSubsystem extends SubsystemBase {
    private static CANSparkMax m_conveyor = new CANSparkMax(ConveyorConstants.kConveyorPort, MotorType.kBrushed);
    private static RelativeEncoder m_conveyorEncoder = m_conveyor.getEncoder(Type.kQuadrature, 2048);

    
    public ConveyorSubsystem() { 
        m_conveyorEncoder.setPosition(0);
        //liftEncoder.setInverted(true);
        //conveyorEncoder.setPositionConversionFactor(10);
        m_conveyor.setInverted(false);
        m_conveyor.setIdleMode(IdleMode.kBrake);
    }
     
    /**
     * @param speed = speed to move at
     */
    public void convey(double speed) {
        m_conveyor.set(speed);
    }

    public void stop(){
        convey(0);
    }

    public double getPose(){
        return m_conveyorEncoder.getPosition();
    }
}
