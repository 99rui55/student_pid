package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class VelocityPIDCommand extends CommandBase {
    Chassis chassis;

    public VelocityPIDCommand(Chassis chassis) {
        this.chassis = chassis;
        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        double v = SmartDashboard.getNumber("Auto Velocity", 0.1);
        chassis.setPID();
        chassis.setVelocity(v);
    }

    @Override
    public void end(boolean interrupted) {
        chassis.setVelocity(0);
    }

}