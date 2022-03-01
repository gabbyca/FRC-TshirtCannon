package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class TurretCommand extends CommandBase {
    TurretSubsystem m_turret;
    private final double m_speed;

    public TurretCommand(TurretSubsystem turret, double speed) {
        m_turret = turret;
        m_speed = speed;
        addRequirements(m_turret);
    }
    @Override
    public void execute() {
        m_turret.rotate(m_speed);
    }
    
}