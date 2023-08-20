package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ChassisConstants;

public class Chassis extends SubsystemBase {

    TalonFX left;
    TalonFX right;

    public Chassis() {
        super();
        left = initMotors(ChassisConstants.LeftFrontMotor, ChassisConstants.LeftBackMotor, ChassisConstants.LeftInverted);
        right = initMotors(ChassisConstants.RightFrontMotor, ChassisConstants.RightBackMotor, ChassisConstants.RightInverted);
    
        SmartDashboard.putData("Chassis",this);
    }

    public void setBrake(){
        Command cmd = new InstantCommand(()-> setBrake(), this);
        SmartDashboard.putData("brake", cmd.ignoringDisable(true));
    }

    public void setCoast(){
        Command cmd = new InstantCommand(()-> setCoast(), this);
        SmartDashboard.putData("coast", cmd.ignoringDisable(true));
    }

    // Init motors for one side
    private TalonFX initMotors(int mainn, int followerr, boolean invert) {
        TalonFX main = new TalonFX(mainn);
        TalonFX follower = new TalonFX(followerr);
        main.setInverted(invert);
        follower.setInverted(invert);
        follower.follow(main);
        setPID(main, ChassisConstants.VelocityKP, ChassisConstants.VelocityKI, ChassisConstants.VelocityKD);
        return main;
    }

    public void setPower(double leftt, double rightt) {
        left.set(ControlMode.PercentOutput, leftt);
        right.set(ControlMode.PercentOutput, rightt);
    }

    public void setVelocity(double leftt, double rightt) {
        // input in meter per seconds
        left.setIntegralAccumulator(0);
        right.setIntegralAccumulator(0);
        left.set(ControlMode.Velocity, VelocityToTalonVelocity(leftt));
        right.set(ControlMode.Velocity, VelocityToTalonVelocity(rightt));
    }
    public void setVelocity(double velocity) {
        setVelocity(velocity, velocity);
    }
    
    public void stop() {
        setPower(0,0);
    }

    private void setPID(TalonFX motor,double kp, double ki, double kd ) {
        motor.config_kP(0, kp);
        motor.config_kI(0, ki);
        motor.config_kD(0, kd);
    }

    public void setPID(double kp, double ki, double kd) {
        setPID(left, kp, ki, kd);
        setPID(right, kp, ki, kd);
    }

    public void setPID() { 
        setPID(SmartDashboard.getNumber("Velocity KP", ChassisConstants.VelocityKP),
                SmartDashboard.getNumber("Velocity KI", ChassisConstants.VelocityKI),
                SmartDashboard.getNumber("Velocity KD", ChassisConstants.VelocityKD));
    }

    // get functions
    public double getLeftDistance() {
        return left.getSelectedSensorPosition()/ChassisConstants.PulsePerMeter;
    }
    public double getRightDistance() {
        return right.getSelectedSensorPosition()/ChassisConstants.PulsePerMeter;
    }
    public double getDistance() {
        return (getLeftDistance() + getRightDistance())/2;
    }
    public double getLeftVelocity() {
        return TalonVelocityToVelocity(left.getSelectedSensorVelocity());
    }
    public double getRightVelocity() {
        return TalonVelocityToVelocity(right.getSelectedSensorVelocity());
    }
    public double getVelocity() {
        return (getLeftVelocity() + getRightVelocity())/2;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Left Distance", this::getLeftDistance, null);
        builder.addDoubleProperty("Right Distance", this::getRightDistance, null);
        builder.addDoubleProperty("Distance", this::getDistance, null);
        builder.addDoubleProperty("Left Velocity", this::getLeftVelocity, null);
        builder.addDoubleProperty("Right Velocity", this::getRightVelocity, null);
        builder.addDoubleProperty("Velocity", this::getVelocity, null);
        SmartDashboard.putNumber("Velocity KP", ChassisConstants.VelocityKP);
        SmartDashboard.putNumber("Velocity KD", ChassisConstants.VelocityKD);
        SmartDashboard.putNumber("Velocity KI", ChassisConstants.VelocityKI);
    }


    // utilities
    public static double TalonVelocityToVelocity(double velocity) {
        return velocity * 10 / ChassisConstants.PulsePerMeter;
    }
    
    public static double VelocityToTalonVelocity(double velocity) {
        return velocity * ChassisConstants.PulsePerMeter / 10;
    }


}
