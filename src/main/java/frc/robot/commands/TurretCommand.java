package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TurretCommand extends CommandBase {
    TurretSubsystem m_turret;
    LimelightSubsystem m_camera;
    private final double m_speed;

    public TurretCommand(TurretSubsystem turret, LimelightSubsystem camera, double speed) {
        m_turret = turret;
        m_camera = camera;
        m_speed = speed;
        addRequirements(m_turret);
    }
    @Override
    public void execute() {
        m_turret.rotate(m_speed + m_camera.getTurnSpeed());
    }
    
}