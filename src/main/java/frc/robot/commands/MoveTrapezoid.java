package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Util.Trapezoid;
import frc.robot.subsystems.Chassis;
import static frc.robot.Constants.ChassisConstants.*;


public class MoveTrapezoid extends CommandBase {
    Chassis chassis;
    Trapezoid trapezoid = new Trapezoid(1, 0.35, 4, 0);
    double remainingDistance = 4;
    double distanceTraveled = 0;
    
    public MoveTrapezoid(Chassis chassis) {
        this.chassis = chassis;
        
        addRequirements(chassis);
    }

    public void moveDistance(double distance) {
        this.remainingDistance = distance;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double originalPos = chassis.getDistance();
        chassis.setVelocity(trapezoid.calculate(remainingDistance, chassis.getVelocity(), 0));
        distanceTraveled = chassis.getDistance() - originalPos;
        remainingDistance -= distanceTraveled;
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
