package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.controller.DifferentialDriveFeedforward;
import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.TalonFXGroup;

import static frc.robot.Constants.ChassisConstants.*;

public class Chassis extends SubsystemBase {
  private final TalonFX leftMotors = new TalonFXGroup(motorFrontLeftId, motorBackLeftId);
  private final TalonFX rightMotors = new TalonFXGroup(motorFrontRightId, motorBackRightId);
  private final PigeonIMU gyro = new PigeonIMU(gyroId);

  private final DifferentialDriveFeedforward ff = new DifferentialDriveFeedforward(kV, kA, kVA, kAA);
  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(
    Rotation2d.fromDegrees(gyro.getFusedHeading()),
    leftMotors.getSelectedSensorPosition() * Constants.pulsesPerMeter,
    leftMotors.getSelectedSensorPosition() * Constants.pulsesPerMeter
  );

  public Chassis() {
    rightMotors.setInverted(true);
  }

  public void setPID(double p) {
    leftMotors.config_kP(0, p);
    rightMotors.config_kP(0, p);
  }

  public void setPID(double p, double i) {
    leftMotors.config_kP(0, p);
    rightMotors.config_kP(0, p);

    leftMotors.config_kI(0, i);
    rightMotors.config_kI(0, i);
  }

  public void setPID(double p, double i, double d) {
    leftMotors.config_kP(0, p);
    rightMotors.config_kP(0, p);

    leftMotors.config_kI(0, i);
    rightMotors.config_kI(0, i);

    leftMotors.config_kD(0, d);
    rightMotors.config_kD(0, d);
  }

  public void setPower(double left, double right) {
    leftMotors.set(ControlMode.PercentOutput, left);
    rightMotors.set(ControlMode.PercentOutput, right);
  }

  public void setVelocity(double left, double right) {
    DifferentialDriveWheelVoltages volts = ff.calculate(getLeftVelocity(), left, getRightVelocity(), right, Constants.cycleTimestep);

    leftMotors.set(ControlMode.Velocity, left, DemandType.ArbitraryFeedForward, volts.left);
    rightMotors.set(ControlMode.Velocity, left, DemandType.ArbitraryFeedForward, volts.right);
  }

  public double getLeftVelocity() {
    return leftMotors.getSelectedSensorVelocity();
  }

  public double getRightVelocity() {
    return rightMotors.getSelectedSensorVelocity();
  }

  @Override
  public void periodic() {
      odometry.update(
        Rotation2d.fromDegrees(gyro.getFusedHeading()),
        leftMotors.getSelectedSensorPosition() * Constants.pulsesPerMeter,
        leftMotors.getSelectedSensorPosition() * Constants.pulsesPerMeter
      );
  }
}
