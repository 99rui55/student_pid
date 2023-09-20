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
    public static final int LeftFrontMotor = 1;
    public static final int LeftBackMotor = 2;
    public static final int RightFrontMotor = 3;
    public static final int RightBackMotor = 4;
    public static final int gyro = 14;
    public static final double PulsePerMeter = 51330;

    public static final double VelocityKP = 0.005;
    public static final double VelocityKI = 0.0005;
    public static final double VelocityKD = 0.00005;  
    public static final double kS = 0.35;  
    public static final double kV = 2.84; 
    public static final double TrackWidth = 0.57; 

    public static final double wantedvelosity = 0.2;  

  }