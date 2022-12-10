// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class ConversionConstants {

  }

  public static final class DriveConstants {
    public static final int 
      FRONT_LEFT_PORT = 4,
      BACK_LEFT_PORT = 1,
      FRONT_RIGHT_PORT = 2,
      BACK_RIGHT_PORT = 3;

    public static final double TRACKWIDTH_METERS = 0.69;
    public static final DifferentialDriveKinematics KINEMATICS =
        new DifferentialDriveKinematics(TRACKWIDTH_METERS);


    // Drive limiters
    public static double
      FORWARD_SCALE = 0.80,
      ROT_SCALE = 0.80;
    
    public static final double
      FORWARD_SCALE_INITIAL = 0.80, // normal scales should be <1.0
      ROT_SCALE_INITIAL = 0.80;

  }

  public static final class OIConstants {
    public static final int 
      DRIVER_PORT = 0,
      INTAKE_PORT = 1;
  }

  public static final class AutoConstants {

    public static final double PEAK_OUTPUT = 0.2;

  }

  public static final class PIDConstants {
    public static final int IDX = 0;
    public static final int TIMEOUT_MS = 30;

    public final static Gains DRIVE_VELOCITY  = new Gains(0.3, 0.00001, 4.0, 0,  0,  0.6);
    public final static Gains INTAKE_POSITION  = new Gains(0.034, 0.00001, 0, 0,  0,  0.25);
  }

}
