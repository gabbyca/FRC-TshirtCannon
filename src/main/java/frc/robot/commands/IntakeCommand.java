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
        if(m_intake.getInches() > 12){
            m_intake.succ(m_speed);
        }
        else{
            m_intake.succ(m_speed);
            Robot.wait(2000);
            m_intake.succ(0);
            m_intake.gotOne();
        }
    }
    
}