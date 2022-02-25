package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiftConstants;

public class LiftSubsystem extends SubsystemBase {
    private static CANSparkMax m_lift = new CANSparkMax(LiftConstants.kLift1Port, MotorType.kBrushed);
    private static RelativeEncoder m_liftEncoder = m_lift.getEncoder(Type.kQuadrature, 2048);
    private CANSparkMax m_lift2 = new CANSparkMax(LiftConstants.kLift2Port, MotorType.kBrushed);
    private PIDController m_pid = new PIDController(LiftConstants.kP, LiftConstants.kI, LiftConstants.kD);

    public LiftSubsystem() { 
        m_liftEncoder.setPosition(0);
        m_liftEncoder.setInverted(true);
        m_liftEncoder.setPositionConversionFactor(10);
        m_lift.setInverted(false);
        m_lift2.follow(m_lift, true);
        m_lift.setIdleMode(IdleMode.kBrake);
        m_lift2.setIdleMode(IdleMode.kBrake);
    }
     
    /**
     * @param speed = speed to move at
     */
    public void moveLift(double pose) {
        m_pid.setSetpoint(pose);
    }

    public void updatePid() {
        m_lift.set(m_pid.calculate(this.getPose()));
    }

    public double getPose(){
        return m_liftEncoder.getPosition();
    }
}