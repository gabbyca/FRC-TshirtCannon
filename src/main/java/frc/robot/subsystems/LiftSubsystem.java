package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiftConstants;
import io.github.oblarg.oblog.annotations.Config;

public class LiftSubsystem extends SubsystemBase {
    private static CANSparkMax m_lift = new CANSparkMax(LiftConstants.kLift1Port, MotorType.kBrushed);
    private static RelativeEncoder m_liftEncoder = m_lift.getEncoder(Type.kQuadrature, 2048);
    private CANSparkMax m_lift2 = new CANSparkMax(LiftConstants.kLift2Port, MotorType.kBrushed);
    @Config
    private PIDController m_pid = new PIDController(LiftConstants.kP, LiftConstants.kI, LiftConstants.kD);

    private double setPoint = 0;
    

    public LiftSubsystem() { 
        m_liftEncoder.setPosition(0);
        m_liftEncoder.setInverted(true);
        m_liftEncoder.setPositionConversionFactor(10);
        m_lift.setInverted(true);
        m_lift2.follow(m_lift, true);
        m_lift.setIdleMode(IdleMode.kBrake);
        m_lift2.setIdleMode(IdleMode.kBrake);
        m_pid.setSetpoint(0);
    }
     
    /**
     * @param speed = speed to move at
     */
    public void moveLift(double pose) {
        m_pid.setSetpoint(pose);
        setPoint = pose;
    }

    public void updatePid() {
        if (setPoint == LiftConstants.topPose)
            m_lift.set(MathUtil.clamp(m_pid.calculate(this.getPose()), -LiftConstants.kMaxLiftPower*0.5,
                    LiftConstants.kMaxLiftPower*0.5));
        else
            m_lift.set(MathUtil.clamp(m_pid.calculate(this.getPose()), -LiftConstants.kMaxLiftPower,
            LiftConstants.kMaxLiftPower));
    }

    public double getPose(){
        return m_liftEncoder.getPosition();
    }
}