package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.utils.Trapezoid;

public class TrapezoidDrive extends CommandBase{
    private ChassisSubsystem chassis;
    private Trapezoid trapezoid;
    private double remainingDistance;
    private double startDistance;
    public TrapezoidDrive(ChassisSubsystem chassis){
        this.chassis = chassis;
        this.trapezoid = new Trapezoid(1, 0.5);
        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        remainingDistance = SmartDashboard.getNumber("Desired distance", 0);
        startDistance = chassis.getDistance();
        System.out.println("starting distance: " + startDistance);
    }
    
    @Override
    public void execute() {
        System.out.println("speed: " + trapezoid.calculate(remainingDistance, chassis.getVelocity(), 0));
        chassis.setVelocity(trapezoid.calculate(remainingDistance, chassis.getVelocity(), 0));
        remainingDistance -= startDistance - chassis.getDistance();
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
