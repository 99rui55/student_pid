// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Drive;
import frc.robot.commands.PIDCommand;
import frc.robot.subsystems.Chassis;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final Chassis chassis;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    chassis = new Chassis();
    configurBindings();
   
  }

  
  private CommandXboxController  controller = new CommandXboxController(0);

  public void configurBindings() {
    controller.b().onTrue(new PIDCommand(chassis).withTimeout(3));
    controller.a().onTrue(new Drive(chassis));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PIDCommand(chassis).withTimeout(3);
    // return null;
  }
}
