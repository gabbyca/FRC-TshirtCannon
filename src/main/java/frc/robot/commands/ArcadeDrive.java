package frc.robot.commands;

import java.util.function.DoubleSupplier;



import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * A command to drvie the robot using mecanum with joystick input 
 * This drives in relation to the field, not the robot itsef
 */

public class ArcadeDrive extends CommandBase {
    private final DriveSubsystem m_drive;
    private final DoubleSupplier m_x;
    private final DoubleSupplier m_y;
    private final DoubleSupplier m_c;
    

    public ArcadeDrive(DriveSubsystem subsystem, DoubleSupplier x, DoubleSupplier y, DoubleSupplier c)
    {
        m_drive = subsystem;
        m_x = x;
        m_y = y;
        m_c = c;
        addRequirements(m_drive);
    }
    @Override
    public void initialize() {
    }
    @Override
    public void execute() {
        m_drive.mecanumDrive(- m_x.getAsDouble() * 0.5 * DriveConstants.speed,
         - m_y.getAsDouble() * 0.5 * DriveConstants.speed,
          m_c.getAsDouble() * 0.5 * DriveConstants.speed);
    }
}