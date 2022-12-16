package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;

import java.lang.Math;

import org.photonvision.PhotonUtils;

public class DriveCommand extends CommandBase {
    private final DriveSubsystem m_drive;
    private final XboxController m_controller;
    private int PIDIter = 0;
    private double forward;
    private double rot;
    
    public DriveCommand(DriveSubsystem subsystem, XboxController controller) {
        m_drive = subsystem;
        addRequirements(m_drive);
        m_controller = controller;
        initializeLimiters();
    }

    private void drive() {
        forward = 
            (m_controller.getRightTriggerAxis() - m_controller.getLeftTriggerAxis())
            * DriveConstants.FORWARD_SCALE;
        rot = m_controller.getLeftX() * DriveConstants.ROT_SCALE;
        double absRotPercent = Math.abs(rot/DriveConstants.ROT_SCALE);

        if (absRotPercent < 0.10) { // deadband
            rot = 0;
        }

        m_drive.updateSpeeds(forward, rot);
        m_drive.drivePID();
    }

    private void updateSmartDashboard() {
        SmartDashboard.putNumber("forward", forward);
        SmartDashboard.putNumber("rotation", rot);
        SmartDashboard.putNumber("Drive position", m_drive.getCurrentPos()[0]);
        SmartDashboard.putNumber("Drive error", m_drive.getCurrentError()[0]);
    }

    private void updateLimiters() {
        DriveConstants.FORWARD_SCALE = SmartDashboard.getNumber("forward scale", DriveConstants.FORWARD_SCALE_INITIAL);
        DriveConstants.ROT_SCALE = SmartDashboard.getNumber("rot scale", DriveConstants.ROT_SCALE_INITIAL);
    }

    private void initializeLimiters() {
        SmartDashboard.putNumber("forward scale", DriveConstants.FORWARD_SCALE);
        SmartDashboard.putNumber("rot scale", DriveConstants.ROT_SCALE);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        updateLimiters();
        drive();
        updateSmartDashboard();
        if(PIDIter*20>5000) { // 5000ms refresh time for PID
            m_drive.updatePIDConstants();
            m_drive.updatePID();
        }
        SmartDashboard.putNumber("ERROR", m_drive.getCurrentError()[0]);
    }
}
