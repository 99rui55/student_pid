package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Chassis extends SubsystemBase {

    TalonFX left;
    TalonFX right;

    TalonFX motorLF = new TalonFX(Constants.LeftFrontMotor);
    TalonFX motorLB = new TalonFX(Constants.LeftBackMotor);
    TalonFX motorRF = new TalonFX(Constants.RightFrontMotor);
    TalonFX motorRB = new TalonFX(Constants.RightBackMotor);
    

    public Chassis() {
        super();
        left = initMotors(Constants.LeftFrontMotor, Constants.LeftBackMotor, Constants.LeftInverted);
        right = initMotors(Constants.RightFrontMotor, Constants.RightBackMotor, Constants.RightInverted);
    
        SmartDashboard.putData("Chassis",this);
    }

    private TalonFX initMotors(int mainn, int followerr, boolean invert) {
        TalonFX main = new TalonFX(mainn);
        TalonFX follower = new TalonFX(followerr);
        main.setInverted(invert);
        follower.setInverted(invert);
        follower.follow(main);
        setPID(main, Constants.KP, Constants.KI, Constants.KD);
        return main;
    }

    public void setPower(double leftt, double rightt) {
        left.set(ControlMode.PercentOutput, leftt);
        right.set(ControlMode.PercentOutput, rightt);
    }

    public void setVelocity(double leftt, double rightt) {
        left.setIntegralAccumulator(0);
        right.setIntegralAccumulator(0);
        left.set(ControlMode.Velocity, leftt * Constants.PulsePerMeter / 10, DemandType.ArbitraryFeedForward, Constants.KV*Math.signum(leftt) + Constants.KS*leftt);
        right.set(ControlMode.Velocity, rightt * Constants.PulsePerMeter / 10, DemandType.ArbitraryFeedForward, Constants.KV*Math.signum(rightt) + Constants.KS*rightt);
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
        setPID(SmartDashboard.getNumber("Velocity KP", Constants.KP),
                SmartDashboard.getNumber("Velocity KI", Constants.KI),
                SmartDashboard.getNumber("Velocity KD", Constants.KD));
    }

    public double getLeftDistance() {
        return left.getSelectedSensorPosition()/Constants.PulsePerMeter;
    }
    public double getRightDistance() {
        return right.getSelectedSensorPosition()/Constants.PulsePerMeter;
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

    public void brake() {
        motorLF.setNeutralMode(NeutralMode.Brake);
        motorLB.setNeutralMode(NeutralMode.Brake);
        motorRF.setNeutralMode(NeutralMode.Brake);
        motorRB.setNeutralMode(NeutralMode.Brake);
    }

    public void coast() {
        motorLF.setNeutralMode(NeutralMode.Coast);
        motorLB.setNeutralMode(NeutralMode.Coast);
        motorRF.setNeutralMode(NeutralMode.Coast);
        motorRB.setNeutralMode(NeutralMode.Coast);
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
        SmartDashboard.putNumber("Velocity KP", Constants.KP);
        SmartDashboard.putNumber("Velocity KD", Constants.KD);
        SmartDashboard.putNumber("Velocity KI", Constants.KI);

        InstantCommand cmdBrake = new InstantCommand(()-> brake(), this);
        SmartDashboard.putData("brake", cmdBrake.ignoringDisable(true));

        InstantCommand cmdCoast = new InstantCommand(()-> coast(), this);
        SmartDashboard.putData("coast", cmdCoast.ignoringDisable(true));
    }

    public static double TalonVelocityToVelocity(double velocity) {
        return velocity * 10 / Constants.PulsePerMeter;
    }
    
}
