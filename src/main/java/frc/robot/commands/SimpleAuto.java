package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

public class SimpleAuto extends CommandBase {
    DriveSubsystem m_drive;

    public SimpleAuto(DriveSubsystem drive) {
        m_drive = drive;
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        m_drive.mecanumDrive(0, 0.3, 0);
        Robot.wait(2000);
        m_drive.mecanumDrive(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    
}
