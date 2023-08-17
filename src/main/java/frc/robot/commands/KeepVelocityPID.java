package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import static frc.robot.Constants.ChassisConstants.*;
import frc.robot.subsystems.ChassisSubsystem;

public class KeepVelocityPID  extends CommandBase{

    private ChassisSubsystem chassis;
    
    public KeepVelocityPID(ChassisSubsystem chassis){
        this.chassis = chassis;
        addRequirements(chassis);
        SmartDashboard.putData(this);
    }

    @Override
    public void initialize() {
        chassis.setVelocity(SmartDashboard.getNumber("Set Velocity", 0));
        System.out.println("Velocity: " + SmartDashboard.getNumber("Set Velocity", 0));
    }
    
    @Override
    public void end(boolean interrupted) {
        chassis.setPower(0, 0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        
    }
    
}
