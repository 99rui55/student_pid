// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Chassis extends SubsystemBase {
  private static TalonFX RFMotor;
  private TalonFX DRMotor;
  private TalonFX DLMotor;
  private TalonFX LFMotor;
  private PigeonIMU gyro;
  public double getRight() {
    double right1 = RFMotor.getSelectedSensorPosition();
    double right2 = DRMotor.getSelectedSensorPosition();
    double right = (right1 + right2)/2;
    return right;
  }
  public double getLeft() {
    double left1 = DLMotor.getSelectedSensorPosition();
    double left2 = LFMotor.getSelectedSensorPosition();
    double left = (left1 + left2)/2;
    return left;
  }

public double present(){
  return RFMotor.getSelectedSensorPosition();
}
public void setPower(double left, double right){
  RFMotor.set(ControlMode.PercentOutput, right);
  DRMotor.set(ControlMode.PercentOutput, right);
  DLMotor.set(ControlMode.PercentOutput, left);
  LFMotor.set(ControlMode.PercentOutput, left);
  
} 
public Chassis() {
  RFMotor = new TalonFX(Constants.uprightmotoridID);
  DRMotor = new TalonFX(Constants.downrightmotoridID);
  DLMotor = new TalonFX(Constants.downleftmotoridID);
  LFMotor = new TalonFX(Constants.upleftmotoridID);
  gyro = new PigeonIMU(Constants.gyroID);
  DLMotor.setInverted(true);
  LFMotor.setInverted(true);
  RFMotor.follow(DRMotor);
  LFMotor.follow(DLMotor);
  DRMotor.config_kP(0,Constants.KP);
  LFMotor.config_kP(0, Constants.KP);
  DRMotor.config_kI(0, Constants.KI);
  LFMotor.config_kI(0, Constants.KI);
  DRMotor.config_kD(0, Constants.KD);
  LFMotor.config_kD(0, Constants.KD);
  SmartDashboard.putNumber("Auto Velocity",1);
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public double GetAngle() {
    return gyro.getFusedHeading();
  }

  public void setvel(double left, double right){
    DRMotor.set(ControlMode.Velocity, right*Constants.PulsePerMeter/10);
    LFMotor.set(ControlMode.Velocity, left*Constants.PulsePerMeter/10);

  }
}

