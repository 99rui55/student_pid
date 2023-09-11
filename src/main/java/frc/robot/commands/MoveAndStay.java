// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Util.Trapezoid;
import frc.robot.subsystems.Chassis;

public class MoveAndStay extends CommandBase{
  private Chassis chassis;
  private Trapezoid trap;
  private double targetDis;
  public MoveAndStay(Chassis chassis, double targetDis){
    addRequirements(chassis);
    this.chassis = chassis;
    this.targetDis = targetDis;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    chassis.resetV();
    targetDis += chassis.getDis();
    double maxV = SmartDashboard.getNumber("Max Velocity", 0.5);
    double maxA = SmartDashboard.getNumber("Max Acceleration", 1);
    trap = new Trapezoid(maxV, maxA);
    System.out.println("maxV = "+ maxV);
    System.out.println("maxA = "+ maxA);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double remainingDis = this.targetDis - chassis.getDis();
    double v = trap.calculate(remainingDis, chassis.getVelocity(), 0);
    chassis.setV(v, v);
    System.out.println("velocity= " + v);
    System.out.println("velocityL= " + chassis.getVelocityL());
    System.out.println("__________________________________");
    System.out.println("velocityR= " + chassis.getVelocityR());
    System.out.println("__________________________________");
    System.out.println("velocity= " + chassis.getVelocity());
    System.out.println("remainingDis = "+ remainingDis);
    System.out.println("dis =  "+ chassis.getDis());
    System.out.println("targetdis = "+ this.targetDis);
    System.out.println("__________________________________");
    System.out.println("neededDis = "+ trap.neededDis());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.setPower(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  @Override
  public void initSendable(SendableBuilder builder) {
      super.initSendable(builder);
  }
}
