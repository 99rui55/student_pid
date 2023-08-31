package frc.robot.subsystems;


import edu.wpi.first.math.controller.DifferentialDriveFeedforward;
import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Chassis extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
    private TalonFX rightFrTalonFX;
    private TalonFX rightBkTalonFX;
    private TalonFX leftBkTalonFX;
    private TalonFX leftFrTalonFx;
    SimpleMotorFeedforward gg = new SimpleMotorFeedforward(Constants.ks, Constants.kv, Constants.ka);
    DifferentialDriveFeedforward gg2 = new DifferentialDriveFeedforward(Constants.kv, Constants.ka, Constants.kva, Constants.kaa, Constants.widthWheels);

    
  /** Creates a new Chassis. */
  public Chassis(){
    super();
    rightFrTalonFX = new TalonFX(Constants.rightFrontMotorID);
    rightBkTalonFX = new TalonFX(Constants.rightBackMotorID);
    leftBkTalonFX = new TalonFX(Constants.leftBackMotorID);
    leftFrTalonFx = new TalonFX(Constants.leftFrontMotorID);
    

    rightFrTalonFX.setInverted(true);
    rightBkTalonFX.setInverted(true);

    rightFrTalonFX.follow(rightBkTalonFX);
    leftFrTalonFx.follow(leftBkTalonFX);

    rightBkTalonFX.config_kP(0, Constants.kp);
    rightBkTalonFX.config_kI(0, Constants.ki);
    rightBkTalonFX.config_kD(0, Constants.kd);
    leftBkTalonFX.config_kP(0, Constants.kp);
    leftBkTalonFX.config_kI(0, Constants.ki);
    leftBkTalonFX.config_kD(0, Constants.kd);
    SmartDashboard.putNumber("KP", Constants.kp);
    SmartDashboard.putNumber("KI", Constants.ki);
    SmartDashboard.putNumber("KD", Constants.kd);
    SmartDashboard.putData("Chassis",this);
    SmartDashboard.putData("Brake",new InstantCommand(
      ()->this.setBrake(),this).ignoringDisable(true));
    SmartDashboard.putData("Coast", new InstantCommand(
      ()->this.setCoast(),this).ignoringDisable(true));
  }


  public void setPower(double leftPower, double rightPower){
    rightFrTalonFX.set(ControlMode.PercentOutput, rightPower);
    rightBkTalonFX.set(ControlMode.PercentOutput, rightPower);
    leftFrTalonFx.set(ControlMode.PercentOutput,leftPower);
    leftBkTalonFX.set(ControlMode.PercentOutput, leftPower);
  }
  public double getVelocityR(){
    return rightBkTalonFX.getSelectedSensorVelocity()
    /Constants.pulsePerMeter*10;
  }
  public double getVelocityL(){
    return leftBkTalonFX.getSelectedSensorPosition()
    /Constants.pulsePerMeter*10;
  }
  public void setV(double vl, double vr){
    double v1 = (vl*Constants.pulsePerMeter)/10;
    double v2 = (vr*Constants.pulsePerMeter)/10;
    DifferentialDriveWheelVoltages volt = gg2.calculate(getVelocityL(), vl, getVelocityR(), vr, Constants.cicleTime);
    rightBkTalonFX.set(ControlMode.Velocity,v2 , DemandType.ArbitraryFeedForward, volt.right/12);
    leftBkTalonFX.set(ControlMode.Velocity,v1 , DemandType.ArbitraryFeedForward, volt.left/12);
  }
  public void setV(double v){
    double v1 = (v*Constants.pulsePerMeter)/10;
    rightBkTalonFX.setIntegralAccumulator(0);
    leftBkTalonFX.setIntegralAccumulator(0);
    double g = gg.calculate(v);
    rightBkTalonFX.set(ControlMode.Velocity,v1 , DemandType.ArbitraryFeedForward, g/12);
    leftBkTalonFX.set(ControlMode.Velocity,v1 , DemandType.ArbitraryFeedForward, g/12);
  }
  
  public void resetV(){
    rightBkTalonFX.setIntegralAccumulator(0);
    leftBkTalonFX.setIntegralAccumulator(0);
  }
  
  @Override
  public void initSendable(SendableBuilder builder) {
      super.initSendable(builder);
      builder.addDoubleProperty("MovementR", this::getRightMeters, null);
      builder.addDoubleProperty("MovementL", this::getLeftMeters, null);
      builder.addDoubleProperty("VelocityR", this::getVelocityR, null);
      builder.addDoubleProperty("VelocityL", this::getVelocityL, null);
      builder.addDoubleProperty("PowerR", this::getRightPower, null);
      builder.addDoubleProperty("PowerL", this::getLeftPower, null);
      builder.addDoubleProperty("Velocity", null, this::setV);
  }
  
  public double getRightPulses(){
    return Math.abs((rightBkTalonFX.getSelectedSensorPosition()+rightFrTalonFX.getSelectedSensorPosition())/2);
  }
  public double getLeftPulses(){
    return Math.abs((leftBkTalonFX.getSelectedSensorPosition()+leftFrTalonFx.getSelectedSensorPosition())/2);
  }
  public double getDis(){
    double pulsePerRotation = 2048;
    double gear = 12;
    double spinPerMeter = 1/(0.15*Math.PI);
    return getRightPulses()/pulsePerRotation/gear/spinPerMeter;
  }

  public double getRightMeters(){
    return getRightPulses()/Constants.pulsePerMeter;
  }
  public double getLeftMeters(){
    return getLeftPulses()/Constants.pulsePerMeter;
  }
  public double getRightPower(){
    return (rightBkTalonFX.getMotorOutputPercent() + rightFrTalonFX.getMotorOutputPercent())/2;
  }
  public double getLeftPower(){
    return (leftBkTalonFX.getMotorOutputPercent() + leftFrTalonFx.getMotorOutputPercent())/2;
  }


  public void setBrake(){
    rightBkTalonFX.setNeutralMode(NeutralMode.Brake);
    rightFrTalonFX.setNeutralMode(NeutralMode.Brake);
    leftBkTalonFX.setNeutralMode(NeutralMode.Brake);
    leftFrTalonFx.setNeutralMode(NeutralMode.Brake);
  }
  public void setCoast(){
    rightBkTalonFX.setNeutralMode(NeutralMode.Coast);
    rightFrTalonFX.setNeutralMode(NeutralMode.Coast);
    leftBkTalonFX.setNeutralMode(NeutralMode.Coast);
    leftFrTalonFx.setNeutralMode(NeutralMode.Coast);
  }



  


  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
