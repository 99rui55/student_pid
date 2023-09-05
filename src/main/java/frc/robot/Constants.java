package frc.robot;

public final class Constants {
  public static class ChassisConstants {
    public static final int motorFrontRightId = 1;
    public static final int motorBackRightId = 2;
    public static final int motorFrontLeftId = 3;
    public static final int motorBackLeftId = 4;
    public static final int gyroId = 14;

    public static final double kV = 2.84 / 12;
    public static final double kA = 0.12 / 12;
    public static final double kVA = 2.85 / 12;
    public static final double kAA = 0.055 / 12;
  }
  public static final double pulsesPerMeter = 51330.6021184;
  public static final double cycleTimestep = 0.02;
}
