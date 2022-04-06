package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.LiftSubsystem;

public class LiftCommand extends CommandBase {
    LiftSubsystem m_lift;
    BlinkinSubsystem m_led;
    private final double m_speed;

    public LiftCommand(LiftSubsystem lift, BlinkinSubsystem led, double speed) {
        m_lift = lift;
        m_led = led;
        m_speed = speed;
        addRequirements(m_lift);
    }
    @Override
    public void execute() {
        m_lift.moveLift(m_speed);
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}
