package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;


public class PIDCommand extends CommandBase {
    Chassis chassis;

    public PIDCommand(Chassis chassis) {
        this.chassis = chassis;
        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        double velocity = SmartDashboard.getNumber("AutoVelocity", 0);
        chassis.setPID();
        chassis.setVelocity(velocity);
    }

    @Override
    public void end(boolean interrupted) {
        chassis.stop();
    }
    
}
