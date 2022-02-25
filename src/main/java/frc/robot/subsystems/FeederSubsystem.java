package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;

public class FeederSubsystem extends SubsystemBase {
    private static CANSparkMax m_feeder = new CANSparkMax(FeederConstants.kFeederPort, MotorType.kBrushed);
    private static RelativeEncoder feederEncoder = m_feeder.getEncoder(Type.kQuadrature, 2048);

    /**
     * this is the lift class we have two motors for the shooting, but since they
     * always follow eachother, we just have lift2 follow shooter also, index is
     * a servo that moves up or down depending on if we want the servo to go in or
     * not
     */
    
    public FeederSubsystem() { 
        feederEncoder.setPosition(0);
        //liftEncoder.setInverted(true);
        //conveyorEncoder.setPositionConversionFactor(10);
        m_feeder.setInverted(true);
        m_feeder.setIdleMode(IdleMode.kBrake);
    }
     
    /**
     * @param speed = speed to move at
     */
    public void feed(double speed) {
        m_feeder.set(speed);
    }

    public double getPose(){
        return feederEncoder.getPosition();
    }
}
