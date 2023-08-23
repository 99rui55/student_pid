package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Chassis extends SubsystemBase {

    TalonFX leftfront;
    TalonFX leftback;
    TalonFX rightfront;
    TalonFX rightback;
    InstantCommand coastcmd;
    InstantCommand brakecmd;

    public Chassis() {
        super();
        leftfront = new TalonFX(Constants.LeftFrontMotor);
        leftback = new TalonFX(Constants.LeftBackMotor);
        rightfront = new TalonFX(Constants.RightFrontMotor);
        rightback = new TalonFX(Constants.RightBackMotor);
        leftback.follow(leftfront);
        rightback.follow(rightfront);
        leftback.setInverted(true);
        leftfront.setInverted(true);
        rightback.setInverted(false);
        rightfront.setInverted(false);
        setPID();
        coastcmd = new InstantCommand(() -> coast());
        brakecmd = new InstantCommand(() -> brake());
        SmartDashboard.putData(this);
    }

    public void setPower(double power) {
        leftfront.set(ControlMode.PercentOutput, power);
        rightfront.set(ControlMode.PercentOutput, power);
    }

    public void setVelocity(double Velocity) {
        leftfront.setIntegralAccumulator(0);
        leftfront.set(ControlMode.Velocity, VelocityToTalonVelocity(Velocity));
        rightfront.setIntegralAccumulator(0);
        rightfront.set(ControlMode.Velocity, VelocityToTalonVelocity(Velocity));
    }
    
    public void setPID() {
        leftfront.config_kP(0, SmartDashboard.getNumber("Velocity KP", Constants.VelocityKP));
        leftfront.config_kI(0, SmartDashboard.getNumber("Velocity KI", Constants.VelocityKI));
        leftfront.config_kD(0, SmartDashboard.getNumber("Velocity KD", Constants.VelocityKD));
        rightfront.config_kP(0, SmartDashboard.getNumber("Velocity KP", Constants.VelocityKP));
        rightfront.config_kI(0, SmartDashboard.getNumber("Velocity KI", Constants.VelocityKI));
        rightfront.config_kD(0, SmartDashboard.getNumber("Velocity KD", Constants.VelocityKD));
    }

    public double getLeftDistance() {
        return leftfront.getSelectedSensorPosition() / Constants.PulsePerMeter;
    }

    public double getRightDistance() {
        return rightfront.getSelectedSensorPosition() / Constants.PulsePerMeter;
    }

    public double getDistance() {
        return (getLeftDistance() + getRightDistance()) / 2;
    }

    public double getLeftVelocity() {
        return TalonVelocityToVelocity(leftfront.getSelectedSensorVelocity());
    }

    public double getRightVelocity() {
        return TalonVelocityToVelocity(leftfront.getSelectedSensorVelocity());
    }

    public double getVelocity() {
        return (getLeftVelocity() + getRightVelocity()) / 2;
    }

    public double getLeftpower() {
        return leftfront.getMotorOutputPercent();
    }

    public double getRightpower() {
        return leftfront.getMotorOutputPercent();
    }

    public double getpower() {
        return (getLeftpower() + getRightpower()) / 2;
    }

    public void coast() {
        leftfront.setNeutralMode(NeutralMode.Coast);
        leftback.setNeutralMode(NeutralMode.Coast);
        rightback.setNeutralMode(NeutralMode.Coast);
        rightfront.setNeutralMode(NeutralMode.Coast);
    }

    public void brake() {
        leftfront.setNeutralMode(NeutralMode.Brake);
        leftback.setNeutralMode(NeutralMode.Brake);
        rightback.setNeutralMode(NeutralMode.Brake);
        rightfront.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Distance right", this::getRightDistance, null);
        builder.addDoubleProperty("Distance left", this::getLeftDistance, null);
        builder.addDoubleProperty("Distance", this::getDistance, null);
        builder.addDoubleProperty("Velocity right", this::getLeftVelocity, null);
        builder.addDoubleProperty("Velocity left", this::getRightVelocity, null);
        builder.addDoubleProperty("Velocity", this::getVelocity, null);
        builder.addDoubleProperty("Power right", this::getRightpower, null);
        builder.addDoubleProperty("Power left", this::getLeftpower, null);
        builder.addDoubleProperty("Power", this::getpower, null);
        SmartDashboard.putNumber("Velocity KP", Constants.VelocityKP);
        SmartDashboard.putNumber("Velocity KI", Constants.VelocityKI);
        SmartDashboard.putNumber("Velocity KD", Constants.VelocityKD);
        SmartDashboard.putNumber("Velocity KD", Constants.VelocityKD);
        SmartDashboard.putNumber("Wanted Velocity", Constants.wantedvelosity);
        SmartDashboard.putNumber("Wanted Velocity", Constants.wantedvelosity);
        SmartDashboard.putData("Coast", coastcmd.ignoringDisable(true));
        SmartDashboard.putData("Brake", brakecmd.ignoringDisable(true));

    }

    public static double TalonVelocityToVelocity(double v) {
        return v * 10 / Constants.PulsePerMeter;
    }

    public static double VelocityToTalonVelocity(double v) {
        return v * Constants.PulsePerMeter / 10;
    }
}
