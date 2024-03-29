// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.Autonomous.ComplexAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  // the default commands
  DriveCommand m_DriveCommand = new DriveCommand(m_robotDrive, m_driverController, false);
  DriveCommand m_DriveCommandPID = new DriveCommand(m_robotDrive, m_driverController, true);
  




  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // max output of drivesubsystem
    m_robotDrive.setMaxOutput(DriveConstants.kDrivePercentDefault);
    

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        m_DriveCommand
    );
  }


  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Drive at half speed when B is held
    new JoystickButton(m_driverController, Button.kB.value)
        .whenPressed(() -> m_robotDrive.setMaxOutput(DriveConstants.kDrivePercentActive))
        .whenReleased(() -> m_robotDrive.setMaxOutput(DriveConstants.kDrivePercentDefault));
    new JoystickButton(m_driverController, Button.kA.value)
        .whenPressed(m_DriveCommandPID)
        .whenReleased(m_DriveCommand);
        // A button is for PID
    } 
    



  /**
   * Use this to pass the  autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */


  public Command getAuto() {
    return new ComplexAuto(m_robotDrive);
  }



}
