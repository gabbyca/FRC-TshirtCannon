package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.BlinkinSubsystem;

public class SpeedControl extends CommandBase {
    BlinkinSubsystem m_led;
    private final double m_speed;

    public SpeedControl(BlinkinSubsystem led, double speed) {
        m_led = led;
        m_speed = speed;
        addRequirements(m_led);
    }
    @Override
    public void execute() {
        if (m_speed == 1)
            m_led.orange();
        else
            m_led.red();

        DriveConstants.speed = m_speed;
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}