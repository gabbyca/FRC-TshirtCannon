package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private static CANSparkMax m_shooter = new CANSparkMax(ShooterConstants.kShooter1Port, MotorType.kBrushed);
    private static RelativeEncoder m_shooterEncoder = m_shooter.getEncoder();
    private CANSparkMax m_shooter2 = new CANSparkMax(ShooterConstants.kShooter2Port, MotorType.kBrushless);
    static PIDController m_pid = new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);

    public ShooterSubsystem() { 
        m_shooter.setInverted(false);
        m_shooter2.follow(m_shooter, true);
    }
     
    /**
     * @param speed = speed to shoot at
     */
    public void shoot(double speed) {
        m_shooter.set(speed);
    }

    public void updatePid() {
        m_shooter.set(m_pid.calculate(this.getVelocity()));
    }

    public void speedUp(){
        while(!m_pid.atSetpoint()){
            updatePid();
        }
    }

    public double getVelocity() {
        return m_shooterEncoder.getVelocity();
    }
}