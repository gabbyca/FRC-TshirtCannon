package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AlignToGoal extends CommandBase {
    DriveSubsystem m_drive;
    LimelightSubsystem m_camera;

    public AlignToGoal(DriveSubsystem drive, LimelightSubsystem camera) {
        m_drive = drive;
        m_camera = camera;
        addRequirements(m_drive, m_camera);
    }
    @Override
    public void execute() {
        while(!(m_camera.getTX() >= -0.5 && m_camera.getTX() <=0.5)){
            m_drive.mecanumDrive(0, 0 , m_camera.getTurnSpeed());
        }
    }
    
}