// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.fasterxml.jackson.databind.deser.impl.ExternalTypeHandler.Builder;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis;

public class DriveSpeed extends CommandBase implements Sendable {
  double v;
  Chassis chassis;  int cycles = 0;
  public double getVelocity(){return v;}
  /** Creates a new DriveSpeed. */
  public DriveSpeed(Chassis chassis) {
    this.chassis = chassis;
    addRequirements(chassis);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  @Override
  public void end(boolean interrupted) {
    chassis.setPower(0, 0);

  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cycles = 0;
    double v = SmartDashboard.getNumber("Auto Velocity", 1);
    
 
}
@Override
public boolean isFinished() {
    return cycles++ > 150;
}

public void Sendable(SendableBuilder builder){
  builder.addDoubleProperty("Velocity", this::getVelocity, null);
}
}
