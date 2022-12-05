package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class ComplexAuto extends SequentialCommandGroup {

    public ComplexAuto(DriveSubsystem drive) {
        addCommands(
            new RunCommand(() -> drive.arcadeDrive(0.6, 0), drive).withTimeout(1.3), // arcade drive is reversed?? TODO: unreverse it
            new RunCommand(() -> drive.arcadeDrive(0, 1), drive).withTimeout(0.6)
        );
    }
    
}
