// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.TalonFXPIDSetConfiguration;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class PIDTask extends CommandBase {
  public Chassis chassis;
  public double KP;
  public double KI;
  public double KD;
  
  /** Creates a new OdiWantPID. */
  public PIDTask(Chassis chassis, double KP, double KI, double KD) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis);
    this.chassis = chassis;

    this.KP = KP;
    this.KI = KI;
    this.KD = KD;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    chassis.invert();
    chassis.setPID(KP, KI, KD);
    // chassis.setVelocity();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  public double wantedSpeed(){return 5;}
  public double speed(double speed){return speed;}
  public void initSendeble(SendableBuilder builder) {
    builder.addDoubleProperty("Wanted Speed", this::wantedSpeed, this::speed);
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