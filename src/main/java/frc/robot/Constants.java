// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.Math;

import javax.swing.JComboBox.KeySelectionManager;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class ChassisConstants {
    public static double voltsPerMeter = 0.417;
    public static final int FrontLeftMotor = 1;
    public static final int BackLeftMotor = 2;
    public static final int FrontRightMotor = 3;
    public static final int BackRightMotor = 3;
    public static final int CountPerMeter = 51330;
    public static final boolean LeftInvert = true;
    public static final boolean RightInvert = false;

    // public static final double Kv = 2.84/12;
    // public static final double Ks = 0.12/12;
    // public static final double Ka = 0.3/12;

    public static final double Kp = (0.417 * 1023) / (12 * CountPerMeter);
    public static final double Kv = 2.84 * voltsPerMeter;
    public static final double Ka = 0.12 * voltsPerMeter;
    public static final double Ks = 0.335 * voltsPerMeter;

    public static final double Kva = 2.85 * voltsPerMeter;
    public static final double Kaa = 0.055 * voltsPerMeter;
    public static final double wheelWidth = 0.58; // Meters
    public static final double circleTime = 0.02; // Seconds
    
    


    public static final double WheelCirc = 6 * 0.0254 * Math.PI;
    public static final double GearRatio = 12;
    public static final double PulsePerRotation = 2048;
    public static final double PulsePerMeter = (1/WheelCirc)*GearRatio*PulsePerRotation;

    public static final double kP = 0.03;
    public static final double kI = 0;
    public static final double kD = 0;

  }
  public static class OperatorConstants {
    public static final int DriverControllerPort = 0;
  }
}
