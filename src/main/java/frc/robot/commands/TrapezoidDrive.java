package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.utils.Trapezoid;

public class TrapezoidDrive extends CommandBase{
    private ChassisSubsystem chassis;
    private Trapezoid trapezoid;
    private double remainingDistance;
    private double targetDistance;
    public TrapezoidDrive(ChassisSubsystem chassis){
        this.chassis = chassis;
        this.trapezoid = new Trapezoid(1, 1);
        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        targetDistance = chassis.getDistance() + SmartDashboard.getNumber("Desired distance", 0);
    }
    
    @Override
    public void execute() {
        remainingDistance = targetDistance - chassis.getDistance();;
        System.out.println("speed: " + trapezoid.calculate(remainingDistance, chassis.getVelocity(), 0));
        chassis.setVelocity(trapezoid.calculate(remainingDistance, chassis.getVelocity(), 0));
        System.out.println("remaining distance: " + remainingDistance);
    }

    @Override
    public boolean isFinished() {
        return remainingDistance <= 0;
    }

    @Override
    public void end(boolean interrupted) {
        chassis.stop();
    }
}
