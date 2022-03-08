package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends CommandBase{
    ShooterSubsystem m_shooter;
    LimelightSubsystem m_camera;
    private final double m_speed;
    

    public ShooterCommand(ShooterSubsystem shooter,LimelightSubsystem camera, double speed) {
        m_shooter = shooter;
        m_camera = camera;
        m_speed = speed;
        addRequirements(m_shooter,  m_camera);
    }
    @Override
    public void execute() {

        m_shooter.shoot(m_speed); // *m_camera.getShooterSpeed()

    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}
