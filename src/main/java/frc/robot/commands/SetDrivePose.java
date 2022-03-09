package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class SetDrivePose extends CommandBase {
    
    DriveSubsystem m_drive;
    Trajectory m_trajectory;
    public SetDrivePose(DriveSubsystem drive, Trajectory trajectory) {
        m_drive = drive;
        m_trajectory = trajectory;
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        m_drive.resetOdometry(m_trajectory.getInitialPose());
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
