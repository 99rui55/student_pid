package frc.robot;

import java.lang.Math;
public final class Constants {
  public static class ChassisConstants {
    public static final int FrontLeftMotor = 1;
    public static final int BackLeftMotor = 1;
    public static final int FrontRightMotor = 1;
    public static final int BackRightMotor = 1;
    public static final boolean LeftInverted = true;
    public static final boolean RightInverted = false;

    public static final double WheelCircumference = 4 * 0.0254 * Math.PI; // 4 Inch wheels
    public static final double GearRatio = 8.14;
    public static final double PulsePerRotation = 2048;
    public static final double PulsePerMeter = (1/WheelCircumference)*GearRatio*PulsePerRotation;

    public static final double VelocityKP = 0.1;
    public static final double VelocityKI = 0.01;
    public static final double VelocityKD = 0;

    public static final String AutoVelocityID = "Auto Velocity";

  }
  public static class OperatorConstants {
    public static final int DriverControllerPort = 0;
  }
}
