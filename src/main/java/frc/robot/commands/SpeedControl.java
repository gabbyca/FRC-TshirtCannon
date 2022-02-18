package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;

public class SpeedControl extends CommandBase {
    private final double m_speed;

    public SpeedControl(double speed) {
        m_speed = speed;
        addRequirements();
    }
    @Override
    public void execute() {
        DriveConstants.speed = m_speed;
    }
    
}