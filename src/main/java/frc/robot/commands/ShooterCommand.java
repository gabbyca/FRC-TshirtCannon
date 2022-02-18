package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends CommandBase{
    ShooterSubsystem m_shooter;
    IntakeSubsystem m_intake;
    LimelightSubsystem m_camera;
    private final double m_speed;
    

    public ShooterCommand(ShooterSubsystem shooter, IntakeSubsystem intake, LimelightSubsystem camera, double speed) {
        m_shooter = shooter;
        m_intake = intake;
        m_camera = camera;
        m_speed = speed;
        addRequirements(m_shooter, m_intake, m_camera);
    }
    @Override
    public void execute() {

        m_shooter.shoot(m_speed); // * m_camera.getShooterSpeed()
        
    }
    
}
