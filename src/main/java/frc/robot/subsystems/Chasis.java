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
import com.ctre.phoenix.sensors.PigeonIMU;


public class Chasis extends SubsystemBase {
  /** Creates a new Chasis. */
  public TalonFX[] motors;
  PigeonIMU pigeon;

  public Chasis(int... id) {
      motors = new TalonFX[id.length];
      for(int i = 0; i < motors.length; i++)
      {
        motors[i] = new TalonFX(id[i]);
        motors[i].config_kP(0, 0.07);
        motors[i].config_kI(0, 0.05);
        motors[i].config_kD(0, 0);
      }

      for(int i = 0; i < motors.length/2;i++)
      {
        motors[i].setInverted(false);
      }
      for(int i = motors.length/2; i < motors.length;i++)
      {
        motors[i].setInverted(true);
      }


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

  public double[] deg()
  {
    double[] ypr = new double[3];
    pigeon.getYawPitchRoll(ypr);
    return ypr;
  }

  public void setLeftP(double p)
  {
      for(int i = 0; i < motors.length/2; i++)
      {
          motors[i].set(ControlMode.PercentOutput, p);
      }
  }

  public void setRightP(double p)
  {
      for(int i = motors.length/2; i < motors.length; i++)
      {
          motors[i].set(ControlMode.PercentOutput, p);
      }
  }

  public double getCountsV(int i)
  {
    return motors[i].getSelectedSensorVelocity();
  }

  public void setRightV(double mPs)
  {
    for(int i = motors.length/2; i < motors.length; i++)
      {
          motors[i].set(ControlMode.Velocity, (mPs * Constants.OperatorConstants.cPerM) / 10);
      }
  }

  public void setLeftV(double mPs)
  {
      for(int i = 0; i < motors.length/2; i++)
      {
          motors[i].set(ControlMode.Velocity, (mPs * Constants.OperatorConstants.cPerM) / 10);
      }
  }

  public double getCounts(int i)
  {
      return motors[i].getSelectedSensorPosition();
  }

  public double getCountsL()
  {
    return motors[motors.length/2-motors.length/4].getSelectedSensorPosition();
  }

  public double getCountsR()
  {
    return motors[motors.length/2+motors.length/4].getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  
}
