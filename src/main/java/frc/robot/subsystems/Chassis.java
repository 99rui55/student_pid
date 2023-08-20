package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ChassisConstants.*;
import frc.robot.RobotContainer;

public class Chassis extends SubsystemBase {

    TalonFX leftMotor = new TalonFX(FrontLeftMotor);
    TalonFX backLeftMotor = new TalonFX(BackLeftMotor);

    TalonFX rightMotor = new TalonFX(FrontRightMotor);
    TalonFX backRightMotor = new TalonFX(BackRightMotor);
    
    RobotContainer container; // for future use


    public Chassis(RobotContainer container) {
        this.container = container;
        initMotors(leftMotor, backLeftMotor, LeftInvert);
        initMotors(rightMotor, backRightMotor, RightInvert);
        setPID();
    }

    // this function initiates two motors
    private void initMotors(TalonFX front, TalonFX back, boolean invert) {
        front.setInverted(invert);
        back.setInverted(invert);
        back.follow(front);
        setPID(front, kP, kI,kD);
    }

    public double return0() {
        return 0;
    }

    public void setPower(double l, double r) {
        leftMotor.set(ControlMode.PercentOutput, l);
        rightMotor.set(ControlMode.PercentOutput, r);
    }

    public void setVelocity(double l, double r) {
        // input in meter per seconds
        leftMotor.setIntegralAccumulator(0);
        rightMotor.setIntegralAccumulator(0);
        leftMotor.set(ControlMode.Velocity, toPulse(l));
        rightMotor.set(ControlMode.Velocity, toPulse(r));
    }
    public void setVelocity(double v) {
        setVelocity(v, v);
    }
    
    public void setBrake() {
        leftMotor.setNeutralMode(NeutralMode.Brake);
        rightMotor.setNeutralMode(NeutralMode.Brake);
        leftMotor.setNeutralMode(NeutralMode.Brake);
        backLeftMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void setCoast() {
        leftMotor.setNeutralMode(NeutralMode.Coast);
        rightMotor.setNeutralMode(NeutralMode.Coast);
        leftMotor.setNeutralMode(NeutralMode.Coast);
        backLeftMotor.setNeutralMode(NeutralMode.Coast);
    }

    public void stop() {
        setPower(0,0);
    }

    private void setPID(TalonFX motor,double kp, double ki, double kd ) {
        motor.config_kP(0, kp);
        motor.config_kI(0, ki);
        motor.config_kD(0, kd);
    }

    public void setPID(double kp, double ki, double kd) {
        setPID(leftMotor, kp, ki, kd);
        setPID(rightMotor, kp, ki, kd);
    }

    public void setPID() { // read PID from network table
    setPID(SmartDashboard.getNumber("Velocity KP", kP),
    SmartDashboard.getNumber("Velocity KI", kI),
                SmartDashboard.getNumber("Velocity KD", kD));
    }

    // get functions
    public double getLeftDistance() {
        return leftMotor.getSelectedSensorPosition()/PulsePerMeter;
    }
    public double getRightDistance() {
        return rightMotor.getSelectedSensorPosition()/PulsePerMeter;
    }
    public double getDistance() {
        return (getLeftDistance() + getRightDistance())/2;
    }
    public double getLeftVelocity() {
        return toVelocity(leftMotor.getSelectedSensorVelocity());
    }
    public double getRightVelocity() {
        return toVelocity(rightMotor.getSelectedSensorVelocity());
    }
    public double getVelocity() {
        return (getLeftVelocity() + getRightVelocity())/2;
    }
    public double getLeftPower() {
        return (leftMotor.getMotorOutputPercent());
    }
    public double getRightPower() {
        return (rightMotor.getMotorOutputPercent());
    }

    // @Override
    // public void initSendable(SendableBuilder builder) {
    //     super.initSendable(builder);
    //     builder.addDoubleProperty("Left Velocity", this::getLeftVelocity, null);
    //     builder.addDoubleProperty("Right Velocity", this::getRightVelocity, null);
    //     builder.addDoubleProperty("Set Velocity", null, null);
    // }


    // Tools
    public static double toVelocity(double pulses) {
        return pulses * 10 / PulsePerMeter;
    }
    
    public static double toPulse(double velocity) {
        return velocity * PulsePerMeter / 10;
    }
    

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Realtime velocity", this::getVelocity, null);
        builder.addDoubleProperty("Set velocity", this::return0, this::setVelocity);

        
        builder.addDoubleProperty("Left Velocity", this::getLeftVelocity, null);
        builder.addDoubleProperty("Right Velocity", this::getRightVelocity, null);
        builder.addDoubleProperty("Left Distance", this::getLeftDistance, null);
        builder.addDoubleProperty("Right Distance", this::getLeftDistance, null);
        builder.addDoubleProperty("Left Power", this::getLeftPower, null);
        builder.addDoubleProperty("Right Power", this::getLeftPower, null);
        SmartDashboard.putData("Brake", new InstantCommand(()->this.setBrake(), this).ignoringDisable(true));
        SmartDashboard.putData("Coast", new InstantCommand(()->this.setCoast(), this).ignoringDisable(true));
    }

}
