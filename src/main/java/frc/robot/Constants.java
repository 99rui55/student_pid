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
  public static class OperatorConstants {
    public static final double maxVolt = 12;
    public static final int kDriverControllerPort = 0;
    public static final double wheelCirc = 6*0.0254*Math.PI;
    public static final double GearRatio = 12;
    public static final double bWheels = 0.58;
    public static final double CountsPerRevolution = 2048;
    public static final double cPerM = (1/wheelCirc)*GearRatio*CountsPerRevolution; //161684.21;
    //public static final double kP = 0.01;
    //public static final double kP = 0.002;
    public static final double kP =  (0.417*1023)/(12*(OperatorConstants.cPerM / 10));
    // public static final double e = 0.417*1023/(12*5133);

    //public static final double kI = 0.0015;
    //public static final double kI = 0.0008;
    //public static final double kI = 0.00065;
    public static final double kI = 0.0;
    //public static final double kD = 0;
    //public static final double kD = 0.0002;
    public static final double kD = 0.0;

    public static final int leftSide = 0;
    public static final int rightSide = 2;

    //public static final double kS = 0.00611;

    public static final double vPerMPS = 0.417;
    //kS, kV, kA, kAA, kVA are in volts
    public static final double kS = 0.3 * vPerMPS;

    public static final double kV = 2.84 * vPerMPS;
    public static final double kA = 0.12 * vPerMPS;

    public static final double kVA = 2.85 * vPerMPS;
    public static final double kAA = 0.055 * vPerMPS;

    public static final double cTime = 0.02;

    //For str8D
    public static final double kPAE = 0.05;

    
  }
}
