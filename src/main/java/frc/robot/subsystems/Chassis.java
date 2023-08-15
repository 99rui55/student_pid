package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.velocityPID;;

public class Chassis extends SubsystemBase {
  private TalonFX  motorRightFront;
  private TalonFX motorRightBack;
  private TalonFX motorLeftFront;
  private TalonFX motorLeftBack;
  private PigeonIMU gyro = new PigeonIMU(14);
  public double velocity;


  public Chassis() {
    motorRightFront = new TalonFX(Constants.rightFrontMotorId);
    motorRightBack = new TalonFX(Constants.rightBackMotorId);
    motorLeftFront = new TalonFX(Constants.leftFrotnMotorId);
    motorLeftBack = new TalonFX(Constants.leftBackMotorId);


    motorLeftBack.follow(motorLeftFront);
    motorRightBack.follow(motorRightFront);

    motorRightFront.config_kP(0, velocityPID.velocityKP);
    motorLeftFront.config_kP(0, velocityPID.velocityKP);
    /* motorRightFront.config_kI(0, velocityPID.velocityKI);
    motorLeftFront.config_kI(0, velocityPID.velocityKI);
    motorRightFront.config_kD(0, velocityPID.velocityKD);
    motorLeftFront.config_kD(0, velocityPID.velocityKD); */




    
  }
  public double getCounts() {
    return motorRightFront.getSelectedSensorPosition();
  }

  public void stop(){
    motorRightFront.set(ControlMode.PercentOutput, 0);
    motorLeftFront.set(ControlMode.PercentOutput, 0);
  }
  
  public void resetAngle() {
    gyro.setFusedHeading(0);
  }
  public double getAngle(){
    return gyro.getFusedHeading();
  }
  public double getVelocity(){
    return (motorRightFront.getSelectedSensorVelocity() / Constants.countPerMeter) * 10 ;

  }
  
  public void setVelocity(double wantedVelocity){

    wantedVelocity = (wantedVelocity * Constants.countPerMeter) / 10;
    motorRightFront.set(TalonFXControlMode.Velocity, wantedVelocity);
  }


  @Override
  public void periodic() {
     

  }
}
