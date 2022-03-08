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
    private static RelativeEncoder m_feederEncoder = m_feeder.getEncoder(Type.kQuadrature, 2048);

    
    public FeederSubsystem() {
        m_feederEncoder.setPosition(0);
        //liftEncoder.setInverted(true);
        //conveyorEncoder.setPositionConversionFactor(10);
        m_feeder.setInverted(true);
        m_feeder.setIdleMode(IdleMode.kBrake);
    }
     
    @Override
    public void periodic() {

    }
    
    /**
     * @param speed = speed to move at
     */
    public void feed(double speed) {
        m_feeder.set(speed);
    }

    public void stop(){
        feed(0);
    }

    public double getPose(){
        return m_feederEncoder.getPosition();
    }
}
