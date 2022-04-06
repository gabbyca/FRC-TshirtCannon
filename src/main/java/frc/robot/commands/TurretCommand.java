package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TurretCommand extends CommandBase {
    TurretSubsystem m_turret;
    LimelightSubsystem m_camera;
    BlinkinSubsystem m_led;
    private final double m_speed;

    public TurretCommand(TurretSubsystem turret, LimelightSubsystem camera, BlinkinSubsystem led, double speed) {
        m_turret = turret;
        m_camera = camera;
        m_led = led;
        m_speed = speed;
        addRequirements(m_turret);
    }
    @Override
    public void execute() {
        if(m_camera.hasTarget() && m_camera.alignedToGoal()){
            m_led.green();
        }
        else if (m_camera.hasTarget()){
            m_led.blue();
        }
        else{
            m_led.red();
        }
        m_turret.rotate(m_speed + m_camera.getTurnSpeed());
    }
    
}