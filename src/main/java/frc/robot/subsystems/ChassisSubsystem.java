package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import  static frc.robot.Constants.ChassisConstants.*;

public class ChassisSubsystem extends SubsystemBase{
    
    public TalonFX rightFrontMotor = new TalonFX(rightFrontMotorID);
    public TalonFX rightBackMotor = new TalonFX(rightBacktMotorID);
    public TalonFX leftFrontMotor = new TalonFX(leftFrontMotorID);
    public TalonFX leftBackMotor = new TalonFX(leftBackMotorID);
    public PigeonIMU gyro = new PigeonIMU(gyroId);

    public ChassisSubsystem() {
        rightFrontMotor.setInverted(RightInverted);
        rightBackMotor.setInverted(RightInverted);
        leftFrontMotor.setInverted(LeftInverted);
        leftBackMotor.setInverted(LeftInverted);
        leftBackMotor.follow(leftFrontMotor);
        rightBackMotor.follow(rightFrontMotor);
        setPID(VelocityKP, VelocityKI, VelocityKD);
    }

    public void setPower(double left, double right){
        rightFrontMotor.set(ControlMode.PercentOutput, right);
        leftFrontMotor.set(ControlMode.PercentOutput, left);
    }

    public void setVelocity(double left, double right){
        leftFrontMotor.setIntegralAccumulator(0);
        rightFrontMotor.setIntegralAccumulator(0);
        rightFrontMotor.set(TalonFXControlMode.Velocity, (right * PulsePerMeter / 10));
        leftFrontMotor.set(TalonFXControlMode.Velocity, left * PulsePerMeter / 10);
    }

    private void setPID(TalonFX motor,double kp, double ki, double kd ) {
        motor.config_kP(0, kp);
        motor.config_kI(0, ki);
        motor.config_kD(0, kd);
    }

    public void setPID(double kp, double ki, double kd) {
        setPID(leftFrontMotor, kp, ki, kd);
        setPID(rightFrontMotor, kp, ki, kd);
        
    }
}
