// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Utils.Trapezoid;
import frc.robot.subsystems.Chassis;

public class TrapezoidCommand extends CommandBase {
  
  Chassis chassis;
  Trapezoid trapezoid;
  double distence;
  double remainingDis;
  double vel;
  double endVel = 0;
  double maxVel;
  double maxAcc;

  /** Creates a new TrapezoidCommand. */
  public TrapezoidCommand(Chassis chassis, double distence) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.chassis = chassis;
    this.distence = distence * Constants.PulsePerMeter;
    addRequirements(chassis);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // for future use: 
    // endVel = SmartDashboard.getNumber("endVel", 0); 
    endVel = 0;
    
    // set up the var
    maxVel = SmartDashboard.getNumber("Max Velocity", 0.5);
    maxAcc = SmartDashboard.getNumber("Max Acceleration", 1);
    distence += chassis.getDistance();
    trapezoid = new Trapezoid(maxVel, maxAcc);
    SmartDashboard.putData(this);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // caculate the velocity needed from the trapezoid caculator
    remainingDis = distence - chassis.getDistance();
    vel = trapezoid.calculate(remainingDis, chassis.getVelocity(), endVel);
    chassis.setVelocity(vel, vel);
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

  // getting to the smart dashboard all of the changing vars
  private String getTrapezoidState(){ return trapezoid.state; }
  private double getRemainingDis(){ return remainingDis; }

  @Override
  public void initSendable (SendableBuilder builder){
    super.initSendable(builder);

    //add the properties
    builder.addStringProperty("Trapezoid State", this::getTrapezoidState, null);
    builder.addDoubleProperty("Remaining Distence", this::getRemainingDis, null);
    SmartDashboard.putNumber("Needed Distence", trapezoid.neededDis);
    SmartDashboard.putNumber("Time", trapezoid.time);

  }

}
