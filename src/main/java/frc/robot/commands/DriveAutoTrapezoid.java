package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;
import frc.robot.utils.Trapezoid;

public class DriveAutoTrapezoid extends CommandBase {
  private final Chassis chassis;

  private Trapezoid trapezoid;

  public DriveAutoTrapezoid(Chassis chassis) {
    this.chassis = chassis;
    addRequirements(chassis);
  }

  @Override
  public void initialize() {
    trapezoid = new Trapezoid(0.4, .1, 2, chassis.getPose().getTranslation());
  }

  @Override
  public void execute() {
    trapezoid.update(chassis.getRightVelocity(), chassis.getPose().getTranslation());
    chassis.setVelocity(trapezoid.getDesiredVelocity(), trapezoid.getDesiredVelocity());
  }

  @Override
  public void end(boolean interrupted) {
    chassis.setPower(0, 0);
  }

  @Override
  public boolean isFinished() {
    return trapezoid.isFinished(chassis.getPose().getTranslation());
  }
}
