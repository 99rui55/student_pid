// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Util.Trapezoid;
import frc.robot.subsystems.Chassis;

public class MoveAndStay extends CommandBase{
  private Chassis chassis;
  public Trapezoid trap;
  private double targetDis;
  private double v;
  public double a;
  public MoveAndStay(Chassis chassis, double v, double a, double targetDis){
    addRequirements(chassis);
    this.chassis = chassis;
    this.v = v;
    this.targetDis = targetDis;
    this.a = a;
    trap = new Trapezoid(v, a);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    chassis.resetV();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double remainingDis = this.targetDis - chassis.getDis();
    double vr = trap.calculate(remainingDis, chassis.getVelocityR(), v);
    
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
