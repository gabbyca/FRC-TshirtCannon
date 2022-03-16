package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class DropIntake extends CommandBase {
    IntakeSubsystem m_intake;
    private final double m_speed;

    public DropIntake(IntakeSubsystem intake, double speed) {
        m_intake = intake;
        m_speed = speed;

        addRequirements(m_intake);
    }
    @Override
    public void execute() {
        m_intake.drop(m_speed);
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}