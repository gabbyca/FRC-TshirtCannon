package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
    IntakeSubsystem m_intake;
    private final double m_speed;

    public IntakeCommand(IntakeSubsystem intake, double speed) {
        m_intake = intake;
        m_speed = speed;
        addRequirements(m_intake);
    }
    @Override
    public void execute() {
        m_intake.succ(m_speed);
        
        
    }
    
}