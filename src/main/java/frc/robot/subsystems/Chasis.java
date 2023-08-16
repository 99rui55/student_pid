// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.constant.Constable;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import static frc.robot.Constants.OperatorConstants;

import com.ctre.phoenix.sensors.PigeonIMU;


public class Chasis extends SubsystemBase {
  /** Creates a new Chasis. */
  public TalonFX[] motors;
  PigeonIMU pigeon;

  public Chasis(int... id) {
    super();
      motors = new TalonFX[id.length];
      for(int i = 0; i < motors.length; i++)
      {
        motors[i] = new TalonFX(id[i]);
        motors[i].config_kP(0, Constants.OperatorConstants.kP);
        motors[i].config_kI(0, Constants.OperatorConstants.kI);
        motors[i].config_kD(0, Constants.OperatorConstants.kD);
      }
      
      //invert the first left motor
      motors[Constants.OperatorConstants.leftSide].setInverted(false);

      //for all other left motors
      for(int i = Constants.OperatorConstants.leftSide + 1; i < Constants.OperatorConstants.rightSide;i++)
      {
        //invert
        motors[i].setInverted(false);
        //follow first motor
        motors[i].follow(motors[Constants.OperatorConstants.leftSide]);
      }

      //invert first right motor
      motors[Constants.OperatorConstants.rightSide].setInverted(true);
      //for all other right motors
      for(int i = (Constants.OperatorConstants.rightSide) + 1; i < motors.length;i++)
      {
        //invert
        motors[i].setInverted(true);
        //follow first motor
        motors[i].follow(motors[Constants.OperatorConstants.rightSide]);
      }

      //reset the I in the PID
      

      pigeon = new PigeonIMU(14);
  }

  public void resetFH()
  {
      pigeon.setFusedHeading(0);
  }

  public double getFH()
  {
      return pigeon.getFusedHeading();
  }

  public void shut()
  {
    motors[Constants.OperatorConstants.leftSide].set(ControlMode.PercentOutput,0);
    motors[Constants.OperatorConstants.rightSide].set(ControlMode.PercentOutput,0);

  }

  public double[] deg()
  {
    double[] ypr = new double[3];
    pigeon.getYawPitchRoll(ypr);
    return ypr;
  }

  public void setLeftP(double p)
  {
      motors[Constants.OperatorConstants.leftSide].set(ControlMode.PercentOutput,p);
  }

  public void setRightP(double p)
  {
    motors[Constants.OperatorConstants.rightSide].set(ControlMode.PercentOutput,p);
  }

  public void setP(double p)
  {
    motors[Constants.OperatorConstants.leftSide].set(ControlMode.PercentOutput,p);
    motors[Constants.OperatorConstants.rightSide].set(ControlMode.PercentOutput,p);
  }

  public double getCountsV(int i)
  {
    return motors[i].getSelectedSensorVelocity();
  }

  public void setRightV(double mPs)
  {
    motors[Constants.OperatorConstants.rightSide].set(ControlMode.Velocity, (mPs * Constants.OperatorConstants.cPerM) / 10);
  }

  public void setLeftV(double mPs)
  {
    TalonFX m = motors[Constants.OperatorConstants.leftSide];
    m.setIntegralAccumulator(0);
    m.set(ControlMode.Velocity, (mPs * Constants.OperatorConstants.cPerM) / 10);
  }

  public void setV(double mPs)
  {
    motors[Constants.OperatorConstants.leftSide].setIntegralAccumulator(0);
    motors[Constants.OperatorConstants.rightSide].setIntegralAccumulator(0);

    motors[Constants.OperatorConstants.leftSide].set(ControlMode.Velocity, (mPs * Constants.OperatorConstants.cPerM) / 10);
    motors[Constants.OperatorConstants.rightSide].set(ControlMode.Velocity, (mPs * Constants.OperatorConstants.cPerM) / 10);
  }

  public double getCounts(int i)
  {
      return motors[i].getSelectedSensorPosition();
  }

  public double getCountsL()
  {
    return motors[Constants.OperatorConstants.leftSide].getSelectedSensorPosition();
  }

  public double getCountsR()
  {
    return motors[Constants.OperatorConstants.rightSide].getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  
}
