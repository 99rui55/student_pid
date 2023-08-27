// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis;

public class FixAngle extends CommandBase {
private double angle;
Chassis chassis;
  public FixAngle() {
    this.chassis = chassis;
    this.angle = angle;
    addRequirements(chassis);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(angle < chassis.GetAngle()-Constants.ChassisConstants.ERROR_RANGE){
      chassis.setPower(0.1, -0.1);
    }else if(chassis.GetAngle() + Constants.ChassisConstants.ERROR_RANGE < angle){
      chassis.setPower(-0.1, 0.1);
    }else if(angle == chassis.GetAngle()){
      chassis.setPower(0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return false; 
  }
}

