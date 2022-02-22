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
    private static RelativeEncoder conveyorEncoder = m_conveyor.getEncoder(Type.kQuadrature, 2048);

    /**
     * this is the lift class we have two motors for the shooting, but since they
     * always follow eachother, we just have lift2 follow shooter also, index is
     * a servo that moves up or down depending on if we want the servo to go in or
     * not
     */
    
    public ConveyorSubsystem() { 
        conveyorEncoder.setPosition(0);
        //liftEncoder.setInverted(true);
        //conveyorEncoder.setPositionConversionFactor(10);
        m_conveyor.setInverted(false);
        m_conveyor.setIdleMode(IdleMode.kBrake);
    }
     
    /**
     * @param speed = speed to move at
     */
    public void liftUp(double speed) {
        m_conveyor.set(speed);
    }

    public double getPose(){
        return conveyorEncoder.getPosition();
    }
}
