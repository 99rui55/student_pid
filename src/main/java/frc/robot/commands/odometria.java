package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Trapez;
import frc.robot.subsystems.Chassis;

public class odometria extends CommandBase {

    static final double AngleKP = -0.5;
    static final double MinDistance = 0.03;
    static final double MinAngle = 3;
    Pose2d tgtPose;
    Chassis chassis;
    double remainingDistance = 0;
    double angleError;
    boolean inInitialTurn = true;
    Trapez Trapez;
    Trapez rotationTrapez = new Trapez(Math.toRadians(90), Math.toRadians(90));

    public odometria(Pose2d pose, double maxVelcotiy, double maxAccelration, Chassis chassis) {
        super();
        tgtPose = new Pose2d(pose.getTranslation(),pose.getRotation()); // copy so data will not change
        this.chassis = chassis;
        Trapez = new Trapez(maxVelcotiy, maxAccelration);
        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Driving from",chassis.getPose().toString());
        SmartDashboard.putString("Driving to",tgtPose.toString());
        inInitialTurn = true;
    }

    @Override
    public void execute() {
        Pose2d pose = chassis.getPose();
        Translation2d vec = tgtPose.getTranslation().minus(pose.getTranslation());
        remainingDistance = vec.getNorm();
        double velocity = Trapez.calculate(remainingDistance, chassis.getVelocity(), 0);
        angleError = vec.getAngle().minus(pose.getRotation()).getDegrees();
        if(inInitialTurn && Math.abs(angleError) > 5) {
            velocity = 0; // turn in place to target
        } else {
            inInitialTurn = false;
        }
        if(remainingDistance < MinDistance) {
            // we reached destination - turn to required end heading
            velocity = 0;
            angleError = tgtPose.getRotation().minus(pose.getRotation()).getDegrees();
        }
        double radiansPerSec = rotationTrapez.calculate(angleError, chassis.getRotationRate(), 0);
        ChassisSpeeds Speed = new ChassisSpeeds(velocity, 0, radiansPerSec);
        chassis.setVelocity(Speed);
    }

    @Override
    public boolean isFinished() {
        return remainingDistance < MinDistance && angleError < MinAngle;
    }

    @Override
    public void end(boolean interrupted) {
        Pose2d p = chassis.getPose();
        SmartDashboard.putString("Driving ended at",p.toString());
        SmartDashboard.putString("Driving error",p.relativeTo(tgtPose).toString() + " dis=" + remainingDistance + " a=" + angleError);
        chassis.setVelocity(0,0);
    }
}