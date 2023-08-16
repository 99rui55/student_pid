package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ChassisConstants.*;
import frc.robot.RobotContainer;

public class Chassis extends SubsystemBase {

    TalonFX left;  // shortcut for leftMotors[0]
    TalonFX right;
    RobotContainer container; // for future use

    public Chassis(RobotContainer container) {
        this.container = container;
        left = initMotors(FrontLeftMotor, LeftBackMotor, LeftInvert);
        right = initMotors(FrontRightMotor, BackRightMotor, RightInvert);
        setPID();
    }

    // Init motors for one side
    private TalonFX initMotors(int main, int follower, boolean invert) {
        TalonFX m = new TalonFX(main);
        TalonFX f = new TalonFX(follower);
        m.setInverted(invert);
        f.setInverted(invert);
        f.follow(m);
        setPID(m, kP, kI,kD);
        return m;
    }

    public void setPower(double l, double r) {
        left.set(ControlMode.PercentOutput, l);
        right.set(ControlMode.PercentOutput, r);
    }

    public void setVelocity(double l, double r) {
        // input in meter per seconds
        left.setIntegralAccumulator(0);
        right.setIntegralAccumulator(0);
        left.set(ControlMode.Velocity, toPulse(l));
        right.set(ControlMode.Velocity, toPulse(r));
    }
    public void setVelocity(double v) {
        setVelocity(v, v);
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
        setPID(left, kp, ki, kd);
        setPID(right, kp, ki, kd);
    }

    public void setPID() { // read PID from network table
    setPID(SmartDashboard.getNumber("Velocity KP", kP),
    SmartDashboard.getNumber("Velocity KI", kI),
                SmartDashboard.getNumber("Velocity KD", kD));
    }

    // get functions
    public double getLeftDistance() {
        return left.getSelectedSensorPosition()/PulsePerMeter;
    }
    public double getRightDistance() {
        return right.getSelectedSensorPosition()/PulsePerMeter;
    }
    public double getDistance() {
        return (getLeftDistance() + getRightDistance())/2;
    }
    public double getLeftVelocity() {
        return toVelocity(left.getSelectedSensorVelocity());
    }
    public double getRightVelocity() {
        return toVelocity(right.getSelectedSensorVelocity());
    }
    public double getVelocity() {
        return (getLeftVelocity() + getRightVelocity())/2;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Left Velocity", this::getLeftVelocity, null);
        builder.addDoubleProperty("Right Velocity", this::getRightVelocity, null);
        builder.addDoubleProperty("Set Velocity", null, null);
    }


    // Tools
    public static double toVelocity(double pulses) {
        return pulses * 10 / PulsePerMeter;
    }
    
    public static double toPulse(double velocity) {
        return velocity * PulsePerMeter / 10;
    }

    // Add Field
    private void addNTBox(String name, double def) {
        if(SmartDashboard.getNumber(name, -1) == -1) {
            SmartDashboard.putNumber(name, def);
        }
    }


}
