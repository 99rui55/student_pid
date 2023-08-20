package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

import  static frc.robot.Constants.ChassisConstants.*;

public class ChassisSubsystem extends SubsystemBase{
    
    TalonFX leftMotors;  // shortcut for leftMotors[0]
    TalonFX rightMotors;
    RobotContainer container;
    PigeonIMU gyro = new PigeonIMU(gyroId);
    private boolean brake = false;
    public ChassisSubsystem(RobotContainer container) {
        this.container = container;
        leftMotors = initMotors(leftFrontMotorID, leftBackMotorID, LeftInverted);
        rightMotors = initMotors(rightFrontMotorID, rightBackMotorID, RightInverted);
        SmartDashboard.putData(this);
    }
    private TalonFX initMotors(int main, int follower, boolean invert) {
        TalonFX m = new TalonFX(main);
        TalonFX f = new TalonFX(follower);
        m.setInverted(invert);
        f.setInverted(invert);
        f.follow(m);
        setPID(m, VelocityKP, VelocityKI, VelocityKD);
        return m;
    }

    public void setPower(double left, double right){
        rightMotors.set(ControlMode.PercentOutput, right);
        leftMotors.set(ControlMode.PercentOutput, left);
    }

    public void stop() {
        setPower(0,0);
    }

    public void setVelocity(double left, double right){
        leftMotors.setIntegralAccumulator(0);
        rightMotors.setIntegralAccumulator(0);
        rightMotors.set(TalonFXControlMode.Velocity, VelocityToTalonVelocity(right));
        leftMotors.set(TalonFXControlMode.Velocity,VelocityToTalonVelocity(left));
    }

    public void setVelocity(double v) {
        setVelocity(v, v);
    }

    private void setPID(TalonFX motor,double kp, double ki, double kd ) {
        motor.config_kP(0, kp);
        motor.config_kI(0, ki);
        motor.config_kD(0, kd);
    }

    public void setPID(double kp, double ki, double kd) {
        setPID(leftMotors, kp, ki, kd);
        setPID(rightMotors, kp, ki, kd);
    }

    public void setPID() { // read PID from network table
        setPID(SmartDashboard.getNumber("Velocity KP", VelocityKP),
                SmartDashboard.getNumber("Velocity KI", VelocityKI),
                SmartDashboard.getNumber("Velocity KD", VelocityKD));
    }

    //getters
    public double getLeftDistance() {
        return leftMotors.getSelectedSensorPosition()/PulsePerMeter;
    }
    public double getRightDistance() {
        return rightMotors.getSelectedSensorPosition()/PulsePerMeter;
    }
    public double getDistance() {
        return (getLeftDistance() + getRightDistance())/2;
    }
    public double getLeftVelocity() {
        return TalonVelocityToVelocity(leftMotors.getSelectedSensorVelocity());
    }
    public double getRightVelocity() {
        return TalonVelocityToVelocity(rightMotors.getSelectedSensorVelocity());
    }
    public double getVelocity() {
        return (getLeftVelocity() + getRightVelocity())/2;
    }
    public double getLeftPower() {
        return leftMotors.getMotorOutputPercent();
    }
    public double getRightPower() {
        return rightMotors.getMotorOutputPercent();
    }
    private boolean getBrake(){return brake;}
   @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        SmartDashboard.putNumber("Desired Velocity", 0);
        builder.addDoubleProperty("Left Distance", this::getLeftDistance, null);
        builder.addDoubleProperty("Right Distance", this::getRightDistance, null);
        builder.addDoubleProperty("Distance", this::getDistance, null);
        builder.addDoubleProperty("Left Velocity", this::getLeftVelocity, null);
        builder.addDoubleProperty("Right Velocity", this::getRightVelocity, null);
        builder.addDoubleProperty("Velocity", this::getLeftVelocity, null);
        builder.addDoubleProperty("RightPower", this::getLeftPower, null);
        builder.addDoubleProperty("LeftPower", this::getRightPower, null);
        SmartDashboard.putNumber("Velocity KP", VelocityKP);
        SmartDashboard.putNumber("Velocity KD", VelocityKD);
        SmartDashboard.putNumber("Velocity KI", VelocityKI);
        SmartDashboard.putData("Brake", brakeCommand);
        SmartDashboard.putData("Coast", coastCommand );
        builder.addBooleanProperty("Brake", this::getBrake, null);
    }
        
    // utilities
     public static double TalonVelocityToVelocity(double v) {
        return v * 10 / PulsePerMeter;
    }
    
    public static double VelocityToTalonVelocity(double v) {
        return v * PulsePerMeter / 10;
    }
    
    //Commands

    CommandBase brakeCommand = new InstantCommand(() -> {
        leftMotors.setNeutralMode(NeutralMode.Brake);
        rightMotors.setNeutralMode(NeutralMode.Brake);
        brake = true;
    })
    {
        @Override
        public boolean runsWhenDisabled() {
            return true;
        }
    };

    CommandBase coastCommand = new InstantCommand(() -> {
        leftMotors.setNeutralMode(NeutralMode.Coast); 
        rightMotors.setNeutralMode(NeutralMode.Coast); 
        brake = false;
    })
    {
        @Override
        public boolean runsWhenDisabled() {
            return true;
        }
    };
}
