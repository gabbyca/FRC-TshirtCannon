package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeSubsystem;

public class DropIntake extends CommandBase {
    IntakeSubsystem m_intake;
    private final double m_speed;
    private final int m_delay;

    public DropIntake(IntakeSubsystem intake, double speed, int delay) {
        m_intake = intake;
        m_speed = speed;
        m_delay = delay;

        addRequirements(m_intake);
    }
    @Override
    public void execute() {
        m_intake.drop(m_speed);
        Robot.wait(m_delay);
        m_intake.drop(0);
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}