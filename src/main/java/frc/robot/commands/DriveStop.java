package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

/**
 * A command to drvie the robot using mecanum with joystick input
 * This drives in relation to the field, not the robot itsef
 */

public class DriveStop extends CommandBase {
    private final DriveSubsystem m_drive;

    public DriveStop(DriveSubsystem subsystem) {
        m_drive = subsystem;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_drive.mecanumDrive(0, 0, 0);
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }

}