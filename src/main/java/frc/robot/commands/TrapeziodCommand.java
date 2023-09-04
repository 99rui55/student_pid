// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis;

public class TrapeziodCommand extends CommandBase {
  public double distence; // distence in meters
  public double pules; // distence in pules
  public double acceleration;
  public Chassis chassis;
  public double maxVelocity;

  public double accelerationTime;
  public double accelerationDistance;

  /** Creates a new TrapeziodCommand. */
  public TrapeziodCommand(double distence, Chassis chassis) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis);
    this.chassis = chassis;
    this.distence = distence;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    acceleration = SmartDashboard.getNumber("acceleration", 0);
    maxVelocity = SmartDashboard.getNumber("maxVel", 0);

    chassis.setPID();

    pules = Constants.PulsePerMeter*distence;
    accelerationTime = maxVelocity/acceleration;
    accelerationDistance = chassis.getVelocity()*accelerationTime + 0.5*acceleration*Math.pow(accelerationTime, 2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (chassis.motorLF.getSelectedSensorPosition() < accelerationDistance){
      chassis.setVelocity(chassis.getVelocity()+acceleration, chassis.getVelocity()+acceleration);
    }

    else if (chassis.motorLF.getSelectedSensorPosition() + accelerationDistance > distence){
      chassis.setVelocity(chassis.getVelocity()-acceleration, chassis.getVelocity()-acceleration);
    }

    else{
      chassis.setVelocity(chassis.getVelocity(), chassis.getVelocity());
    }
  }

  public double getDistance(){ return distence; }
  public double getPules() { return pules; }
  public double getAccelerationDistance() { return accelerationDistance; }
  public double getAccelerationTime() { return accelerationTime; }

  @Override
  public void initSendable(SendableBuilder builder) {
      super.initSendable(builder);

      builder.addDoubleProperty("distance", this::getDistance, null);
      builder.addDoubleProperty("pules", this::getPules, null);
      builder.addDoubleProperty("acceleration distence", this::getAccelerationDistance, null);
      builder.addDoubleProperty("acceleration time", this::getAccelerationTime, null);
      

  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
