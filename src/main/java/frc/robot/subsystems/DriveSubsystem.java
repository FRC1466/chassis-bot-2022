// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PIDConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class DriveSubsystem extends SubsystemBase {

  // motor array in a list for easy access (could do dict in future?)
  WPI_TalonFX[] motors = new WPI_TalonFX[] {
    new WPI_TalonFX(DriveConstants.FRONT_LEFT_PORT),  // motors[0] = left1
    new WPI_TalonFX(DriveConstants.BACK_LEFT_PORT),  // motors[1] = left2
    new WPI_TalonFX(DriveConstants.FRONT_RIGHT_PORT), // motors[2] = right1
    new WPI_TalonFX(DriveConstants.BACK_RIGHT_PORT)  // motors[3] = right2
  }; 

  ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);
  DifferentialDriveWheelSpeeds wheelSpeeds = DriveConstants.KINEMATICS.toWheelSpeeds(speeds);

  private void initializeMotors() { //set configs of motors
    for (int i=0; i<motors.length; i++) {
      motors[i].configFactoryDefault();
      motors[i].set(ControlMode.PercentOutput, 0);
      motors[i].setNeutralMode(NeutralMode.Brake);
      motors[i].configNeutralDeadband(0.001);
      if (i < motors.length/2)
      {
        motors[i].setInverted(TalonFXInvertType.Clockwise);
      } else {
        motors[i].setInverted(TalonFXInvertType.CounterClockwise);
      }
    }
  }
  

  private void initializePID() { //set configs of PID
    for (WPI_TalonFX motor : motors) 
    {
      motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
      PIDConstants.IDX, 
      PIDConstants.TIMEOUT_MS);


      /* Config the peak and nominal outputs */
      motor.configNominalOutputForward(0, PIDConstants.TIMEOUT_MS);
      motor.configNominalOutputReverse(0, PIDConstants.TIMEOUT_MS);
      motor.configPeakOutputForward(PIDConstants.DRIVE_VELOCITY.PEAK_OUTPUT, PIDConstants.TIMEOUT_MS);
      motor.configPeakOutputReverse(-PIDConstants.DRIVE_VELOCITY.PEAK_OUTPUT, PIDConstants.TIMEOUT_MS);

      /* Config the Velocity closed loop gains in slot0 */
      motor.config_kF(PIDConstants.IDX, PIDConstants.DRIVE_VELOCITY.F, PIDConstants.TIMEOUT_MS);
      motor.config_kP(PIDConstants.IDX, PIDConstants.DRIVE_VELOCITY.P, PIDConstants.TIMEOUT_MS);
      motor.config_kI(PIDConstants.IDX, PIDConstants.DRIVE_VELOCITY.I, PIDConstants.TIMEOUT_MS);
      motor.config_kD(PIDConstants.IDX, PIDConstants.DRIVE_VELOCITY.D, PIDConstants.TIMEOUT_MS);
    }
  }

  public void updatePID() {
    for (WPI_TalonFX motor : motors) {
      motor.config_kF(PIDConstants.IDX, PIDConstants.DRIVE_VELOCITY.F, PIDConstants.TIMEOUT_MS);
      motor.config_kP(PIDConstants.IDX, PIDConstants.DRIVE_VELOCITY.P, PIDConstants.TIMEOUT_MS);
      motor.config_kI(PIDConstants.IDX, PIDConstants.DRIVE_VELOCITY.I, PIDConstants.TIMEOUT_MS);
      motor.config_kD(PIDConstants.IDX, PIDConstants.DRIVE_VELOCITY.D, PIDConstants.TIMEOUT_MS);
    }
  }

  public void updatePIDConstants() {
    PIDConstants.DRIVE_VELOCITY.F = SmartDashboard.getNumber("vel_F", PIDConstants.DRIVE_VELOCITY.F);
    PIDConstants.DRIVE_VELOCITY.P = SmartDashboard.getNumber("vel_P", PIDConstants.DRIVE_VELOCITY.P);
    PIDConstants.DRIVE_VELOCITY.I = SmartDashboard.getNumber("vel_I", PIDConstants.DRIVE_VELOCITY.I);
    PIDConstants.DRIVE_VELOCITY.D = SmartDashboard.getNumber("vel_D", PIDConstants.DRIVE_VELOCITY.D);
  }

  private void initializePIDConstants() {
    SmartDashboard.putNumber("vel_F", PIDConstants.DRIVE_VELOCITY.F);
    SmartDashboard.putNumber("vel_P", PIDConstants.DRIVE_VELOCITY.P);
    SmartDashboard.putNumber("vel_I", PIDConstants.DRIVE_VELOCITY.I);
    SmartDashboard.putNumber("vel_D", PIDConstants.DRIVE_VELOCITY.D);
  }

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    initializeMotors();
    initializePID();
    initializePIDConstants();
  }

  @Override
  public void periodic() {

  }

  /**
   * Drives the robot using PID velocity
   */
  public void drivePID() {
    for (int i=0; i<(motors.length/2); i++) { // left
      motors[i].set(TalonFXControlMode.Velocity, wheelSpeeds.leftMetersPerSecond);
    }
    for (int i=(motors.length/2); i<motors.length; i++) { // right
      motors[i].set(TalonFXControlMode.Velocity, wheelSpeeds.rightMetersPerSecond);
    }
    SmartDashboard.putNumber("left speeds", wheelSpeeds.leftMetersPerSecond);
    SmartDashboard.putNumber("right speeds", wheelSpeeds.rightMetersPerSecond);
  }

  public void updateSpeeds(double vx, double rot) {
    speeds.vxMetersPerSecond = vx;
    speeds.omegaRadiansPerSecond = rot;
    wheelSpeeds = DriveConstants.KINEMATICS.toWheelSpeeds(speeds);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    for (WPI_TalonFX wpi_TalonFX : motors) {
      wpi_TalonFX.setVoltage(0);
    }
  }

  /**
   * Gets current position of motors in a list
   * @return current position of motors in a list
   */
  public double[] getCurrentPos() {
    double[] motorPos = {
      motors[0].getSelectedSensorPosition(),
      motors[1].getSelectedSensorPosition(),
      motors[2].getSelectedSensorPosition(),
      motors[3].getSelectedSensorPosition()
    };
    return motorPos;
  }

  /**
   * Gets current error of motors in a list
   * @return current error of motors in a list
   *
   */
  public double[] getCurrentError() {
    double[] motorPos = {
      motors[0].getClosedLoopError(),
      motors[1].getClosedLoopError(),
      motors[2].getClosedLoopError(),
      motors[3].getClosedLoopError()
    };
    return motorPos;
  }

  /**
   * Gets current target of motors in a list
   * @return current target of motors in a list
   *
   */
  public double[] getCurrentTarget() {
    double[] currentTarget = {
      motors[0].getClosedLoopTarget(),
      motors[1].getClosedLoopTarget(),
      motors[2].getClosedLoopTarget(),
      motors[3].getClosedLoopTarget()
    };
    return currentTarget;
  }

  /**
   * Sets the maximum output of the PID in motors
   * @param peakOutput the peak output
   */
  public void setPeakOutputPID(double peakOutput) {
    for (int i = 0; i < motors.length; i++) {
      motors[i].configPeakOutputForward(peakOutput);
      motors[i].configPeakOutputReverse(-peakOutput);
    }
  }

}
