package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LiftSubsystem;

public class LiftCommand extends CommandBase {
    LiftSubsystem m_intake;
    private final double m_speed;

    public LiftCommand(LiftSubsystem lift, double speed) {
        m_lift = lift;
        m_speed = speed;
        addRequirements(m_intake);
    }
    @Override
    public void execute() {
        m_lift.liftUp(m_speed);
        
        
    }
    
}
