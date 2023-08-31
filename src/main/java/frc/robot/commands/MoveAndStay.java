// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis;

public class MoveAndStay extends CommandBase{
  private Chassis chassis;
  private double v;
  PigeonIMU gyro = new PigeonIMU(14);
  private double startAngle;
  private double x;
  double error = 0;
  public MoveAndStay(Chassis chassis, double v, double x) {
    addRequirements(chassis);
    this.chassis = chassis;
    this.v = v;
    this.startAngle = gyro.getFusedHeading();
    this.x = x;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    chassis.setV(v, v);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentAngle = gyro.getFusedHeading();
    if(currentAngle > startAngle + 3){
      chassis.setV(v,v-1);
    }
    if(gyro.getFusedHeading() < startAngle - 3){
      chassis.setV(v-1,v);
      
    }
    if((gyro.getFusedHeading() <= startAngle + 3)&&(gyro.getFusedHeading() >= startAngle - 3)){
      chassis.setV(v,v);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.setPower(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((chassis.getRightPulses() - error)/Constants.pulsePerMeter >=x){
      return true;
    }
    return false;
  }
  @Override
  public void initSendable(SendableBuilder builder) {
      super.initSendable(builder);
  }
}
