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
  public static class ChassisConstants{
    // Motors
    public static final int rightFrontMotorID = 1;
    public static final int rightBackMotorID = 2;
    public static final int leftFrontMotorID = 3;
    public static final int leftBackMotorID = 4;
    public static final boolean LeftInverted = false;
    public static final boolean RightInverted = true;
    
    public static final double WheelCircumference = 4 * 0.0254 * Math.PI; // 4 Inch wheels
    public static final double GearRatio = 8.14;
    public static final double PulsePerRotation = 2048;
    public static final double PulsePerMeter = (1/WheelCircumference)*GearRatio*PulsePerRotation;
    
    public static final int gyroId = 14;  
    public static final int fixedAngleErrorRange = 3;
    
    // Velocity PID
    public static final double VelocityKP = 50.0/5300.0;
    public static final double VelocityKI = 0;//VelocityKP/10
    public static final double VelocityKD = 0;//VelocityKI/10;

    // FF
    public static final double Ks = 0.3;
    public static final double Kv = 2.84;
    public static final double Ka = 0.12;
    public static final double Kp = 0.006975;
    public static final double Kva = 2.85;
    public static final double Kaa = 0.55;
    public static final double TW = 0.58;
    
    public static class OperatorConstants {
      // Controller
      public static final int xboxControllerID = 0;
      public static final double deadbandValue = 0.1;
    }
  }



  




}
