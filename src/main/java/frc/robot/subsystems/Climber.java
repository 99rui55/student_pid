/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.*;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  // Components of the climber.
  private final DoubleSolenoid m_solenoid;

  // Working variables.
  //private boolean emergencyOff = false;

  /**
   * Represents the climber of Team 1895's robot. It handles the raising,
   * lowering, and locking of the elevator to allow climbing on the power switch
   * at the end of a match.
   */
  public Climber() {

    this.m_solenoid = new DoubleSolenoid(null, CLIMBER_CHANNEL_A, CLIMBER_CHANNEL_B);



    Shuffleboard.getTab("Climber").add("Subsystem", this);
    Shuffleboard.getTab("Climber").add("Solenoid", m_solenoid);
  }

  public void Pistonsopen() {
    this.m_solenoid.set(DoubleSolenoid.Value.kForward);
  }
  public void Pistonscloased() {
    this.m_solenoid.set(DoubleSolenoid.Value.kReverse);
  }

  @Override
  public void periodic() {
  }
}