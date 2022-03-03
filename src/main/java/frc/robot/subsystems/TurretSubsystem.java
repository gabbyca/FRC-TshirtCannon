package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
    private static CANSparkMax m_turret = new CANSparkMax(TurretConstants.kTurretPort, MotorType.kBrushed);
    private static RelativeEncoder m_turretEncoder = m_turret.getEncoder(Type.kQuadrature, 2048);

    public TurretSubsystem() { 
        m_turretEncoder.setPosition(0);
        m_turret.setInverted(false);
        m_turret.setIdleMode(IdleMode.kBrake);
    }
     

    public void rotate(double speed) {
        m_turret.set(speed);
    }

    public void stop(){
        rotate(0);
    }

    public double getPose(){
        return m_turretEncoder.getPosition();
    }
}
