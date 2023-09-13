// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class sec3vel extends CommandBase {
  /** Creates a new sec3vel. */
  Chasis chasis;
  double speed;
  double reqSpeed = 1;
  int time = 0;

  public sec3vel(Chasis chasis) {
    // Use addRequirements() here to declare subsystem dependencies.
    super();
    this.chasis = chasis;
    addRequirements(this.chasis);
    SmartDashboard.putData(this);
  }

  public double getReqSpeed() {
    return this.reqSpeed;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Meters per second", this::getReqSpeed, this::setReqSpeed);
    // builder.addDoubleProperty("Current mPs", this::getSpeed, null);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    chasis.setV(reqSpeed);
  }

  public void setReqSpeed(double reqSpeed) {
    this.reqSpeed = reqSpeed;
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    time++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chasis.shut();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return time >= 150/* Three seconds */;
  }
}
