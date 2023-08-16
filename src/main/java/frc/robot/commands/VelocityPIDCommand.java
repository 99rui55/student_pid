package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;
import static frc.robot.Constants.ChassisConstants.*;


public class VelocityPIDCommand extends CommandBase {
    Chassis chassis;
    private double velocity;

    public void setVelocity(double velocity) {
        this.velocity = velocity;
    }

    public double return0() {
        return 0;
    }

    public VelocityPIDCommand(Chassis chassis) {
        this.chassis = chassis;
        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        chassis.setVelocity(velocity);
    }

    @Override
    public void end(boolean interrupted) {
        chassis.stop();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Realtime velocity", chassis::getVelocity, null);
        builder.addDoubleProperty("Set velocity", this::return0, this::setVelocity);
    }
}
