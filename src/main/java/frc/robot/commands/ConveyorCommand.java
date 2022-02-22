package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConveyorSubsystem;

public class ConveyorCommand extends CommandBase {
    ConveyorSubsystem m_conveyor;
    private final double m_speed;

    public ConveyorCommand(ConveyorSubsystem conveyor, double speed) {
        m_conveyor = conveyor;
        m_speed = speed;
        addRequirements(m_conveyor);
    }
    @Override
    public void execute() {
        m_conveyor.convey(m_speed);
    }
    
}