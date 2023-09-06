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
        this.trapezoid = new Trapezoid(1, 2);
        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        remainingDistance = SmartDashboard.getNumber("Desired distance", 0);
        startDistance = chassis.getDistance();
    }
    
    @Override
    public void execute() {
        chassis.setVelocity(trapezoid.calculate(remainingDistance, chassis.getVelocity(), 0));
        remainingDistance -= startDistance - chassis.getDistance();
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
