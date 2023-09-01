// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.*;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chasis;
public class str8D extends CommandBase {
  /** Creates a new str8D. */
  speedCtrl SC;
  Chasis chasis;

  double tgtVel;
  double maxVel;
  double maxAcc;
  double distance;

  public str8D(double distance,double tgtVel,double maxAcc, double maxVel) {
    
    this.SC = new speedCtrl(maxVel,maxAcc);
    this.maxAcc = maxAcc;
    this.maxVel = maxVel;
    this.tgtVel = tgtVel;
    this.distance = distance;

    this.chasis = new Chasis(4, 3, 2, 1);
    addRequirements(chasis);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double vel = SC.calculate(distance - chasis.getCountsAvgM(), chasis.getSpeedAvg(), tgtVel, 0);
    double angFix = effErrA(chasis.deg()[0]) * OperatorConstants.kPAE;
    chasis.setV(vel + angFix, vel - angFix);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return distance - chasis.getCountsAvgM() > 0;
  }

  public double effErrA(double deg)
  {
    if(deg < 180)
      return -deg;
    else
      return 360 - deg;

  }
}
