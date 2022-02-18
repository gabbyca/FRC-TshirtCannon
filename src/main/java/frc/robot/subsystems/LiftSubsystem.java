package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiftConstants;

public class LiftSubsystem extends SubsystemBase {
    private static CANSparkMax m_lift = new CANSparkMax(LiftConstants.kLift1Port, MotorType.kBrushed);
    private static RelativeEncoder liftEncoder = m_lift.getEncoder(Type.kQuadrature, 2048);
    private CANSparkMax m_lift2 = new CANSparkMax(LiftConstants.kLift2Port, MotorType.kBrushed);

    /**
     * this is the lift class we have two motors for the shooting, but since they
     * always follow eachother, we just have lift2 follow shooter also, index is
     * a servo that moves up or down depending on if we want the servo to go in or
     * not
     */
    
    public LiftSubsystem() { 
        liftEncoder.setPosition(0);
        liftEncoder.setInverted(true);
        liftEncoder.setPositionConversionFactor(10);
        m_lift.setInverted(false);
        m_lift2.follow(m_lift, true);
        m_lift.setIdleMode(IdleMode.kBrake);
        m_lift2.setIdleMode(IdleMode.kBrake);
    }
     
    /**
     * @param speed = speed to move at
     */
    public void liftUp(double speed) {
        m_lift.set(speed);
    }

    public double getPose(){
        return liftEncoder.getPosition();
    }
}