// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DrivePIDConstants {
    public static final int defaultSlot = 0;
    public static final int smartMotionSlot = 1;
    public static final int smartVelocitySlot = 2;

    public static final double kP = 0.00025;
    public static final double kI = 0.000000;
    public static final double kD = 0.0000000;
    public static final double kIz = 0;
    public static final double kFF = 0;
    public static final double kMaxOutput = 1;
    public static final double kMinOutput = -1;
    public static final double maxRPM = 5700;
    public static final double maxVel = 3000;
    public static final double minVel = 0;
    public static final double maxAcc = 4000;
    public static final double allowedErr = 1;

    public static final double kPsv = 0.00025;
    public static final double kIsv = 0.000000000;
    public static final double kDsv = 0.00000000;
    public static final double kIzsv = 0;
    public static final double kFFsv = 0;
    public static final double kMaxOutputsv = 1;
    public static final double kMinOutputsv = -1;
    public static final double maxRPMsv = 5700;
    public static final double maxVelsv = 5700;
    public static final double minVelsv = 0;
    public static final double maxAccsv = 3500;
    public static final double allowedErrsv = 0;
  }

  public static class PhysicalConstants {
    public static final double kRobotWidth = 19.5;
  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}

