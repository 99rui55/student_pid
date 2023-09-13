package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;
import static frc.robot.Constants.ChassisConstants.*;


public class MoveSimple extends CommandBase {
    Chassis chassis;
    private double velocity = 0.5;

    public MoveSimple(Chassis chassis) {
        this.chassis = chassis;
        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        System.out.println("setting velocity to " + velocity);
        chassis.setVelocity(velocity);
    }

    @Override
    public void end(boolean interrupted) {
        chassis.stop();
    }
}
