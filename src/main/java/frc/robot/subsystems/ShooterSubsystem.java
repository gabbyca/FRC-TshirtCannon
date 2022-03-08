package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;
import io.github.oblarg.oblog.annotations.Config;

public class ShooterSubsystem extends SubsystemBase {
    private static Servo m_angle = new Servo(ShooterConstants.kServoPort);
    private static CANSparkMax m_shooter = new CANSparkMax(ShooterConstants.kShooter1Port, MotorType.kBrushless);
    private static RelativeEncoder m_shooterEncoder = m_shooter.getEncoder();
    private CANSparkMax m_shooter2 = new CANSparkMax(ShooterConstants.kShooter2Port, MotorType.kBrushless);
    @Config
    static ProfiledPIDController m_pid = new ProfiledPIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD,
        ShooterConstants.constraints);
    public static final SimpleMotorFeedforward m_ff =
        new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV, ShooterConstants.kA);

    public double setpoint = 0;

    public ShooterSubsystem() {
        m_shooter.setIdleMode(IdleMode.kCoast);
        m_shooter2.setIdleMode(IdleMode.kCoast);
        m_shooter.setInverted(false);
        m_shooter2.follow(m_shooter, true);
        m_pid.setGoal(0);
        m_shooterEncoder.setPosition(0);
        m_shooterEncoder.setPositionConversionFactor(1);
        m_shooterEncoder.setVelocityConversionFactor(1);
        m_angle.set(0);
    }
     
    @Override
    public void periodic() {
        updatePid();
    }
    /**
     * @param speed = speed to shoot at
     */
    public void shoot(double speed) {
        m_pid.setGoal(speed);
        m_pid.setConstraints(ShooterConstants.constraints);
        setpoint = speed;

    }

    public void stop(){
        shoot(0);
    }

    public void updatePid() {
        if(setpoint != 0)
            m_shooter.setVoltage(m_pid.calculate(this.getVelocity()) + ShooterConstants.kF);

        else
        m_shooter.set(0);

    }

    public void changeAngle(double value){
        m_angle.set(value);
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