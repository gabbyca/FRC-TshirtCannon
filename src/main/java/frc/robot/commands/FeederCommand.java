package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;

public class FeederCommand extends CommandBase {
    FeederSubsystem m_feeder;
    private final double m_speed;

    public FeederCommand(FeederSubsystem feeder, double speed) {
        m_feeder = feeder;
        m_speed = speed;
        addRequirements(m_feeder);
    }
    @Override
    public void execute() {
        m_feeder.feed(m_speed);
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}
