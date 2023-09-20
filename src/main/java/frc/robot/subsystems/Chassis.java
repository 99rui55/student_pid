package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Chassis extends SubsystemBase{

    TalonFX leftfront;
    TalonFX leftback;
    TalonFX rightfront;
    TalonFX rightback;
    PigeonIMU gyro;
    Pose2d pose;
    DifferentialDriveKinematics kinematics;
    public Chassis() {
      super();
        leftfront = new TalonFX(Constants.LeftFrontMotor);
        leftback = new TalonFX(Constants.LeftBackMotor);
        rightfront = new TalonFX(Constants.RightFrontMotor);
        rightback = new TalonFX(Constants.RightBackMotor);
        leftback.follow(leftfront);
        rightback.follow(rightfront);
        leftback.setInverted(true);
        leftfront.setInverted(true);
        rightback.setInverted(false);
        rightfront.setInverted(false);
        gyro = new PigeonIMU(Constants.gyro);
        setPID();
        pose = new Pose2d(0,0,getGyroAngle());
        SmartDashboard.putData(this);

    }
    public void setPower(double power) {
        leftfront.set(ControlMode.PercentOutput, power);
        rightfront.set(ControlMode.PercentOutput, power);
    }
    public void setVelocity(double leftVelocity, double rightVelocity) {
        leftfront.setIntegralAccumulator(0);
        leftfront.set(ControlMode.Velocity, leftVelocity * Constants.PulsePerMeter / 10., DemandType.ArbitraryFeedForward, Constants.kS + Constants.kV * leftVelocity);
        rightfront.setIntegralAccumulator(0);
        rightfront.set(ControlMode.Velocity, rightVelocity * Constants.PulsePerMeter / 10., DemandType.ArbitraryFeedForward, Constants.kS + Constants.kV * rightVelocity);
    }
    public void setPID() {
        leftfront.config_kP(0, SmartDashboard.getNumber("Velocity KP", Constants.VelocityKP));
        leftfront.config_kI(0, SmartDashboard.getNumber("Velocity KI", Constants.VelocityKI));
        leftfront.config_kD(0, SmartDashboard.getNumber("Velocity KD", Constants.VelocityKD));
        rightfront.config_kP(0, SmartDashboard.getNumber("Velocity KP", Constants.VelocityKP));
        rightfront.config_kI(0, SmartDashboard.getNumber("Velocity KI", Constants.VelocityKI));
        rightfront.config_kD(0, SmartDashboard.getNumber("Velocity KD", Constants.VelocityKD));
    }
    public double getDistance() {
        return (leftfront.getSelectedSensorPosition()/Constants.PulsePerMeter + rightfront.getSelectedSensorPosition()/Constants.PulsePerMeter)/2;
    }
    public double getVelocity() {
        return (TalonVelocityToVelocity(leftfront.getSelectedSensorVelocity()) + TalonVelocityToVelocity(rightfront.getSelectedSensorVelocity()))/2;
    }
    public double getrotation(){
        return gyro.getFusedHeading();
      }

    
    @Override
    public void initSendable(SendableBuilder builder) {
      super.initSendable(builder);
        builder.addDoubleProperty("Distance", this::getDistance, null);
        builder.addDoubleProperty("Velocity", this::getVelocity, null);
        SmartDashboard.putNumber("Velocity KP", Constants.VelocityKP);
        SmartDashboard.putNumber("Velocity KI", Constants.VelocityKI);
        SmartDashboard.putNumber("Velocity KD", Constants.VelocityKD);
        SmartDashboard.putNumber("Velocity KD", Constants.VelocityKD);
        SmartDashboard.putNumber("Wanted Velocity", Constants.wantedvelosity);
      
    }
    public static double TalonVelocityToVelocity(double v) {
        return v * 10 / Constants.PulsePerMeter;
    }
    public static double VelocityToTalonVelocity(double v) {
        return v * Constants.PulsePerMeter / 10;
    }
    public Rotation2d getGyroAngle() {
        return Rotation2d.fromDegrees(gyro.getFusedHeading());
    }
    public double heading() {
        return pose.getRotation().getDegrees();
    }
    public Pose2d getPose() {
        return pose;
    }
    public double getRotationRate() {
        double[] rates = new double[3]; // x,y,z rates
        gyro.getRawGyro(rates);
        return rates[2]; // z
    }
    public void setVelocity(DifferentialDriveWheelSpeeds speeds) {
        setVelocity(speeds.leftMetersPerSecond,speeds.rightMetersPerSecond);
    }
    public void setVelocity(ChassisSpeeds speed) {
        setVelocity(kinematics.toWheelSpeeds(speed));
    }
}