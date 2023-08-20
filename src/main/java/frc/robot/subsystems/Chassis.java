package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.velocityPID;;

public class Chassis extends SubsystemBase {
  private TalonFX motorRightFront;
  private TalonFX motorRightBack;
  private TalonFX motorLeftFront;
  private TalonFX motorLeftBack;
  private PigeonIMU gyro = new PigeonIMU(14);
  public double velocity;

  public Chassis() {
    super();

    motorRightFront = new TalonFX(Constants.rightFrontMotorId);
    motorRightBack = new TalonFX(Constants.rightBackMotorId);
    motorLeftFront = new TalonFX(Constants.leftFrotnMotorId);
    motorLeftBack = new TalonFX(Constants.leftBackMotorId);

    motorLeftBack.follow(motorLeftFront);
    motorRightBack.follow(motorRightFront);

    motorRightFront.config_kP(0, velocityPID.velocityKP);
    motorLeftFront.config_kP(0, velocityPID.velocityKP);
    SmartDashboard.putData(this);
    /*
     * motorRightFront.config_kI(0, velocityPID.velocityKI);
     * motorLeftFront.config_kI(0, velocityPID.velocityKI);
     * motorRightFront.config_kD(0, velocityPID.velocityKD);
     * motorLeftFront.config_kD(0, velocityPID.velocityKD);
     */

  }

  public double getCounts() {
    return motorRightFront.getSelectedSensorPosition();
  }

  public void stop() {
    motorRightFront.set(ControlMode.PercentOutput, 0);
    motorLeftFront.set(ControlMode.PercentOutput, 0);
  }

  public void resetAngle() {
    gyro.setFusedHeading(0);
  }

  public double getAngle() {
    return gyro.getFusedHeading();
  }

  public double getVelocityRight() {
    return (motorRightFront.getSelectedSensorVelocity() / Constants.countPerMeter) * 10;
  }

  public double getVelocityLeft() {
    return (motorLeftFront.getSelectedSensorVelocity() / Constants.countPerMeter) * 10;
  }

  public void setVelocity(double wantedVelocity) {

    wantedVelocity = (wantedVelocity * Constants.countPerMeter) / 10;
    motorRightFront.set(TalonFXControlMode.Velocity, wantedVelocity);
  }

  public double getPositionRight() {
    return motorRightFront.getSelectedSensorPosition() / Constants.countPerMeter;

  }

  public double getPositionLeft() {
    return motorLeftFront.getSelectedSensorPosition() / Constants.countPerMeter;
  }

  public double getPowerRight() {
    return motorRightFront.getMotorOutputPercent();
  }

  public double getPowerLeft() {
    return motorLeftFront.getMotorOutputPercent();
  }

  public void setBreak() {
    motorRightFront.setNeutralMode(NeutralMode.Brake);
    motorRightBack.setNeutralMode(NeutralMode.Brake);
    motorLeftFront.setNeutralMode(NeutralMode.Brake);
    motorLeftBack.setNeutralMode(NeutralMode.Brake);
  }

  public void setCoast() {
    motorRightFront.setNeutralMode(NeutralMode.Coast);
    motorRightBack.setNeutralMode(NeutralMode.Coast);
    motorLeftFront.setNeutralMode(NeutralMode.Coast);
    motorLeftBack.setNeutralMode(NeutralMode.Coast);
  }

  InstantCommand setBrakeCommand = new InstantCommand(() -> setBreak(), this);
  InstantCommand setCoastCommand = new InstantCommand(() -> setCoast(), this);

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    SmartDashboard.putData("Brake", setBrakeCommand.ignoringDisable(true));
    SmartDashboard.putData("Coast", setCoastCommand.ignoringDisable(true));

    SmartDashboard.putNumber("Velocity Right", getVelocityRight());
    SmartDashboard.putNumber("Velocity Left", getVelocityLeft());
    SmartDashboard.putNumber("Power Right", getPowerRight());
    SmartDashboard.putNumber("Power Left", getPowerLeft());
    SmartDashboard.putNumber("Position Right", getPositionRight());
    SmartDashboard.putNumber("Position Left", getPositionLeft());
  }

  @Override
  public void periodic() {

  }
}
