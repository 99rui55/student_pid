package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.DriveAuto;
import frc.robot.commands.DriveAutoTrapezoid;
import frc.robot.subsystems.Chassis;

public class RobotContainer {
  Chassis chassis = new Chassis();

  public RobotContainer() {
    SmartDashboard.putData("Trapezoid Command", new DriveAutoTrapezoid(chassis));
    SmartDashboard.putData("3 Second Command", new DriveAuto(chassis).withTimeout(3).andThen(
      new InstantCommand(()->{System.out.println("COMMAND ENDED------------------------------------------------");})
    ));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
