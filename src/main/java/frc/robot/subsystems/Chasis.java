// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.DifferentialDriveFeedforward;
import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.util.sendable.SendableBuilder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.Constants;
import static frc.robot.Constants.OperatorConstants;

import com.ctre.phoenix.sensors.PigeonIMU;

public class Chasis extends SubsystemBase {
  /** Creates a new Chasis. */
  public TalonFX[] motors;
  PigeonIMU pigeon;
  DifferentialDriveFeedforward ddff;

  public Chasis(int... id) {
    super();

    this.ddff = new DifferentialDriveFeedforward(OperatorConstants.kV, OperatorConstants.kA, OperatorConstants.kVA,OperatorConstants.kAA);

    motors = new TalonFX[id.length];
    for (int i = 0; i < motors.length; i++) {
      motors[i] = new TalonFX(id[i]);
      motors[i].config_kP(0, Constants.OperatorConstants.kP);
      motors[i].config_kI(0, Constants.OperatorConstants.kI);
      motors[i].config_kD(0, Constants.OperatorConstants.kD);
    }

    // invert the first left motor
    motors[Constants.OperatorConstants.leftSide].setInverted(false);

    // for all other left motors
    for (int i = Constants.OperatorConstants.leftSide + 1; i < Constants.OperatorConstants.rightSide; i++) {
      // invert
      motors[i].setInverted(false);
      // follow first motor
      motors[i].follow(motors[Constants.OperatorConstants.leftSide]);
    }

    // invert first right motor
    motors[Constants.OperatorConstants.rightSide].setInverted(true);
    // for all other right motors
    for (int i = (Constants.OperatorConstants.rightSide) + 1; i < motors.length; i++) {
      // invert
      motors[i].setInverted(true);
      // follow first motor
      motors[i].follow(motors[Constants.OperatorConstants.rightSide]);
    }

    pigeon = new PigeonIMU(14);

    SmartDashboard.putData(this);

    WrapperCommand brakes = new InstantCommand(() -> this.brakes(), this).ignoringDisable(true);

    SmartDashboard.putData("Breaks", brakes);

    WrapperCommand coast = new InstantCommand(() -> this.coast(), this).ignoringDisable(true);

    SmartDashboard.putData("Coast", coast);

  }

  public void resetFH() {
    pigeon.setFusedHeading(0);
  }

  public double getFH() {
    return pigeon.getFusedHeading();
  }

  public void shut() {
    motors[Constants.OperatorConstants.leftSide].set(ControlMode.PercentOutput, 0);
    motors[Constants.OperatorConstants.rightSide].set(ControlMode.PercentOutput, 0);

  }

  public double[] deg() {
    double[] ypr = new double[3];
    pigeon.getYawPitchRoll(ypr);
    return ypr;
  }

  public void setLeftP(double p) {
    motors[Constants.OperatorConstants.leftSide].setIntegralAccumulator(0);
    motors[Constants.OperatorConstants.leftSide].set(ControlMode.PercentOutput, p);
  }

  public void setRightP(double p) {
    motors[Constants.OperatorConstants.rightSide].setIntegralAccumulator(0);
    motors[Constants.OperatorConstants.rightSide].set(ControlMode.PercentOutput, p);
  }

  public void setP(double p) {
    motors[Constants.OperatorConstants.leftSide].setIntegralAccumulator(0);
    motors[Constants.OperatorConstants.rightSide].setIntegralAccumulator(0);

    motors[Constants.OperatorConstants.leftSide].set(ControlMode.PercentOutput, p);
    motors[Constants.OperatorConstants.rightSide].set(ControlMode.PercentOutput, p);
  }

  public double getCountsV(int i) {
    return motors[i].getSelectedSensorVelocity();
  }

  public void setRightV(double mPs) {
    motors[Constants.OperatorConstants.rightSide].setIntegralAccumulator(0);
    DifferentialDriveWheelVoltages volt =  ddff.calculate(
    this.getSpeedL(),
    this.getSpeedL(),
    this.getSpeedR(),
     mPs,
     OperatorConstants.cTime);
    
    double pRight = (volt.right + OperatorConstants.kS*Math.signum(mPs))/OperatorConstants.maxVolt;
    motors[OperatorConstants.rightSide].set(ControlMode.PercentOutput,pRight);
    //motors[Constants.OperatorConstants.rightSide].set(ControlMode.Velocity,
    //    (mPs * Constants.OperatorConstants.cPerM) / 10);
  }

  public void setLeftV(double mPs) {
    motors[Constants.OperatorConstants.leftSide].setIntegralAccumulator(0);
    DifferentialDriveWheelVoltages volt =  ddff.calculate(
    this.getSpeedL(),
    mPs,
    this.getSpeedR(),
    this.getSpeedR(),
     OperatorConstants.cTime);
    
    double pLeft = (volt.left + OperatorConstants.kS*Math.signum(mPs))/OperatorConstants.maxVolt;
    motors[OperatorConstants.rightSide].set(ControlMode.PercentOutput,pLeft);
  }

  public void setV(double mPs) {
    motors[OperatorConstants.leftSide].setIntegralAccumulator(0);
    motors[OperatorConstants.rightSide].setIntegralAccumulator(0);

    // motors[Constants.OperatorConstants.leftSide].set(ControlMode.Velocity,
    //     (mPs * Constants.OperatorConstants.cPerM) / 10);
    // motors[Constants.OperatorConstants.rightSide].set(ControlMode.Velocity,
    //     (mPs * Constants.OperatorConstants.cPerM) / 10);
    DifferentialDriveWheelVoltages volt =  ddff.calculate(
    this.getSpeedL(),
     mPs,
    this.getSpeedR(),
     mPs,
     OperatorConstants.cTime);
    
    double pLeft = (volt.left + OperatorConstants.kS*Math.signum(mPs))/OperatorConstants.maxVolt;
    double pRight = (volt.right + OperatorConstants.kS*Math.signum(mPs))/OperatorConstants.maxVolt;

    motors[OperatorConstants.leftSide].set(ControlMode.PercentOutput,pLeft);
    motors[OperatorConstants.rightSide].set(ControlMode.PercentOutput,pRight);

    

    // motors[OperatorConstants.leftSide].set(
    //   ControlMode.Velocity,
    //   mPs * OperatorConstants.cPerM,
    //   DemandType.ArbitraryFeedForward,
    //   (OperatorConstants.kS*Math.signum(mPs) + OperatorConstants.kV*mPs)/12);
      
  }

  public void setV(double mPsLeft, double mPsRight) {
    motors[OperatorConstants.leftSide].setIntegralAccumulator(0);
    motors[OperatorConstants.rightSide].setIntegralAccumulator(0);

    // motors[Constants.OperatorConstants.leftSide].set(ControlMode.Velocity,
    //     (mPs * Constants.OperatorConstants.cPerM) / 10);
    // motors[Constants.OperatorConstants.rightSide].set(ControlMode.Velocity,
    //     (mPs * Constants.OperatorConstants.cPerM) / 10);
    DifferentialDriveWheelVoltages volt =  ddff.calculate(
    this.getSpeedL(),
     mPsLeft,
    this.getSpeedR(),
     mPsRight,
     OperatorConstants.cTime);
    
     double pLeft = (volt.left + OperatorConstants.kS*Math.signum(mPsLeft))/OperatorConstants.maxVolt;
     double pRight = (volt.right + OperatorConstants.kS*Math.signum(mPsRight))/OperatorConstants.maxVolt;

    motors[OperatorConstants.leftSide].set(ControlMode.PercentOutput,pLeft);
    motors[OperatorConstants.rightSide].set(ControlMode.PercentOutput,pRight);
      
  }

  public double getCounts(int i) {
    return motors[i].getSelectedSensorPosition();
  }

  public double getCountsM(int i)
  {
    return getCounts(i) / OperatorConstants.cPerM;
  }

  public double getCountsL() {
    return motors[Constants.OperatorConstants.leftSide].getSelectedSensorPosition();
  }

  public double getCountsR() {
    return motors[Constants.OperatorConstants.rightSide].getSelectedSensorPosition();
  }

  public double getCountsLM() {
    return getCountsL() / OperatorConstants.cPerM;
  }

  public double getCountsRM() {
    return getCountsR() / OperatorConstants.cPerM;
  }

  public double getCountsAvg()
  {
    return (getCountsR() + getCountsL())/2;
  }

  public double getCountsAvgM()
  {
    return getCountsAvg() / OperatorConstants.cPerM;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Current mPs L", this::getSpeedL, null);
    builder.addDoubleProperty("Current mPs R", this::getSpeedR, null);
    builder.addDoubleProperty("Current mPs R", this::getSpeedAvg, null);

    builder.addDoubleProperty("Counts L", this::getCountsL, null);
    builder.addDoubleProperty("Counts R", this::getCountsR, null);
    builder.addDoubleProperty("Counts Avg", this::getCountsAvg, null);

  }

  public double getSpeedL() {
    return (this.getCountsV(OperatorConstants.leftSide) * 10) / Constants.OperatorConstants.cPerM;
  }

  public double getSpeedR() {
    return (this.getCountsV(OperatorConstants.rightSide) * 10) / Constants.OperatorConstants.cPerM;
  }
  public double getSpeedAvg() {
    return (getSpeedL() + getSpeedR())/2;
  }

  public void printVelocity() {
    System.out.println("Velocity = " + getSpeedAvg());
  }

  public void brakes() {
    System.out.println("Breaks");
    for (int i = 0; i < motors.length; i++) {
      this.motors[i].setNeutralMode(NeutralMode.Brake);
    }

  }

  public void coast() {
    System.out.println("Coast");
    for (int i = 0; i < motors.length; i++) {
      this.motors[i].setNeutralMode(NeutralMode.Coast);
    }

  }
}
