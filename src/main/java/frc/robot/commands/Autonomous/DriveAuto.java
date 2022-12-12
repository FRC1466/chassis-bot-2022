package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveAuto extends CommandBase {
    private DriveSubsystem m_drive;
    
    public DriveAuto(DriveSubsystem drive) {
        m_drive = drive;
        
    }   
}
