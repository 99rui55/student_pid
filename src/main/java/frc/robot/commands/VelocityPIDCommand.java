package frc.robot.commands;

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

    // @Override
    // public void initSendable(SendableBuilder builder) {
    //     super.initSendable(builder);
    //     builder.addDoubleProperty("Realtime velocity", chassis::getVelocity, null);
    //     builder.addDoubleProperty("Set velocity", this::return0, this::setVelocity);

    //     builder.addBooleanProperty("Coast", false, null);
    //     builder.addBooleanProperty("Brake", false, null);

        
    //     builder.addDoubleProperty("Left Velocity", chassis::getLeftVelocity, null);
    //     builder.addDoubleProperty("Right Velocity", chassis::getRightVelocity, null);
    //     builder.addDoubleProperty("Left Distance", chassis::getLeftDistance, null);
    //     builder.addDoubleProperty("Right Distance", chassis::getLeftDistance, null);
    //     builder.addDoubleProperty("Left Power", chassis::getLeftPower, null);
    //     builder.addDoubleProperty("Right Power", chassis::getLeftPower, null);
    // }
}
