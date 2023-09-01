package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import static frc.robot.Constants.ChassisConstants.*;
import frc.robot.subsystems.ChassisSubsystem;

public class KeepVelocityPID  extends CommandBase{

    private ChassisSubsystem chassis;
    
    public KeepVelocityPID(ChassisSubsystem chassis){
        this.chassis = chassis;
        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        chassis.setVelocity(SmartDashboard.getNumber("Final Desired Velocity", 1));
        chassis.setPID();
        System.out.println("HI Bill! " + SmartDashboard.getNumber("Final Desired Velocity", 1));
    }
    
    @Override
    public void end(boolean interrupted) {
        chassis.stop();;
    }
}
