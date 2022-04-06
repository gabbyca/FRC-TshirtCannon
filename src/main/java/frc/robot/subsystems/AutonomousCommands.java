package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class AutonomousCommands extends SubsystemBase {

    DriveSubsystem m_drive;
    TrajectorySubsystem m_trajectories;
    
    public AutonomousCommands(DriveSubsystem drive, TrajectorySubsystem trajectories) {
        m_drive = drive;
        m_trajectories = trajectories;
    }
    
    MecanumControllerCommand m_leftPickUp = new MecanumControllerCommand(
            m_trajectories.getLeftPickUp(),
            m_drive::getPose,
            DriveConstants.kFeedforward,
            DriveConstants.kDriveKinematics,
            // Position contollers
            DriveConstants.xController,
            DriveConstants.yController,
            DriveConstants.thetaController,
            // Needed for normalizing wheel speeds
            DriveConstants.kMaxSpeedMetersPerSecond,
            // Velocity PID's
            DriveConstants.driveController,
            DriveConstants.driveController,
            DriveConstants.driveController,
            DriveConstants.driveController,
            m_drive::getCurrentWheelSpeeds,
            m_drive::setDriveMotorControllersVolts, // Consumer for the output motor voltages
            m_drive);
    
}
