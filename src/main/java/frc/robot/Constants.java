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
  public static final int rightFrontMotorID = 1;
  public static final int rightBackMotorID = 2;
  public static final int leftFrontMotorID = 3;
  public static final int leftBackMotorID = 4;

  public static final double widthWheels = 0.58;
  public static final double cicleTime = 0.02;

  public static final double pulsePerMeter = 51339;

  public static final double kp = 0.417*1023/(12*5133);
  public static final double ki = 0.0000;
  public static final double kd = 0.00;

  public static final double kv = 2.84/12;
  public static final double ka = 0.12/12;
  public static final double ks = 0.3/12;
  public static final double kva = 2.85/12;
  public static final double kaa = 0.055/12;
  
  public static final double sp = 1.5;
}
