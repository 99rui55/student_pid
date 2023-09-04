package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Chassis extends SubsystemBase {
  private final TalonFX motorFrontRight;
  private final TalonFX motorBackRight;
  private final TalonFX motorFrontLeft;
  private final TalonFX motorBackLeft;

  public Chassis() {
    motorFrontRight = new TalonFX(Constants.motorFrontRightId);
    motorBackRight = new TalonFX(Constants.motorBackRightId);
    motorFrontLeft = new TalonFX(Constants.motorFrontLeftId);
    motorBackLeft = new TalonFX(Constants.motorBackLeftId);

    motorBackRight.follow(motorFrontRight);
    motorBackLeft.follow(motorFrontLeft);

    motorFrontLeft.setInverted(true);
    motorBackLeft.setInverted(true);
  }

  public void modifyMotorConfig(double p) {
    motorFrontRight.config_kP(0, p);
    motorFrontLeft.config_kP(0, p);
  }

  public void setVelocity(double right, double left) {
    motorFrontRight.set(ControlMode.Velocity, right * Constants.pulsesPerMeter * 0.1);
    motorFrontLeft.set(ControlMode.Velocity, left * Constants.pulsesPerMeter * 0.1);
  }

  public double getVelocity() {
    return motorFrontRight.getSelectedSensorVelocity() * Constants.pulsesPerMeter * 10;
  }
}
