package frc.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.Constants;
import frc.RobotMap;

public class Shooter extends Subsystem {
  private final WPI_TalonSRX ShooterMaster = new WPI_TalonSRX(RobotMap.FLYWHEEL_MASTER);
  // private final WPI_TalonSRX ShooterFollower = new WPI_TalonSRX(RobotMap.FLYWHEEL_FOLLOWER);

  private static Shooter instance = null;
  private double mCurrentVelocitySetpoint;

  private Shooter() {
    ShooterMaster.configFactoryDefault();
    // ShooterFollower.configFactoryDefault();

    ShooterMaster.setNeutralMode(NeutralMode.Coast);
    // ShooterFollower.setNeutralMode(NeutralMode.Coast);

    ShooterMaster.enableVoltageCompensation(true);
    // ShooterFollower.enableVoltageCompensation(true);

    ShooterMaster.configVoltageCompSaturation(10.0);
    // ShooterFollower.configVoltageCompSaturation(10.0);
    
    ShooterMaster.setInverted(true);
    // ShooterFollower.setInverted(true);

    ShooterMaster.config_kP(0, Constants.Shooter.FLYWHEEL_KP);
    ShooterMaster.config_kI(0, Constants.Shooter.FLYWHEEL_KI);
    ShooterMaster.config_kD(0, Constants.Shooter.FLYWHEEL_KD);
    ShooterMaster.config_kF(0, Constants.Shooter.FLYWHEEL_KF);

    // ShooterFollower.config_kP(0, 0.15);
    // ShooterFollower.config_kI(0, 0.0);
    // ShooterFollower.config_kD(0, 0.0);
    // ShooterFollower.config_kF(0, 0.0570);

    // ShooterMaster.configVelocityMeasurementPeriod(10, 10);    	
    // ShooterMaster.configVelocityMeasurementWindow(32, 10);

    ShooterMaster.setStatusFramePeriod(StatusFrame.Status_1_General, 40);
    // ShooterFollower.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
    ShooterMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10);
    // ShooterFollower.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 500);
    ShooterMaster.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 500);
    // ShooterFollower.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 500);
    ShooterMaster.setStatusFramePeriod(StatusFrame.Status_6_Misc, 500);
    // ShooterFollower.setStatusFramePeriod(StatusFrame.Status_6_Misc, 500);
    ShooterMaster.setStatusFramePeriod(StatusFrame.Status_7_CommStatus, 500);
    // ShooterFollower.setStatusFramePeriod(StatusFrame.Status_7_CommStatus, 500);
    ShooterMaster.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 500);
    // ShooterFollower.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 500);
    ShooterMaster.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 500);
    // ShooterFollower.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 500);
    ShooterMaster.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 500);
    // ShooterFollower.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 500);
    ShooterMaster.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 500);
    // ShooterFollower.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 500);
    ShooterMaster.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 500);
    // ShooterFollower.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 500);
    ShooterMaster.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 500);
    // ShooterFollower.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 500);
    ShooterMaster.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 500);
    // ShooterFollower.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 500);

  }

  public static Shooter getInstance() {
    if (instance == null)
      instance = new Shooter();
    return instance;
  }

  public void setMotor(double speed) {
    if(speed <= Constants.Shooter.MINIMUM_SHOOTER_SPEED) {
      speed = Constants.Shooter.MINIMUM_SHOOTER_SPEED;
      ShooterMaster.set(speed);
    }
    else {
      ShooterMaster.set(speed);
    }
  }

  public void setOpenLoop(final double speed) {
    ShooterMaster.set(ControlMode.PercentOutput, speed);
    // ShooterFollower.set(ControlMode.PercentOutput, speed);
  }

  public void setVelocity(final double velocity){
    mCurrentVelocitySetpoint = velocity;
    ShooterMaster.set(ControlMode.Velocity, velocity);
    // ShooterFollower.set(ControlMode.Velocity, velocity);
  }

  public boolean atTargetVelocity(){
    return Math.abs(ShooterMaster.getSelectedSensorVelocity() - mCurrentVelocitySetpoint) < Constants.Shooter.SHOOTER_VELOCITY_TOLERANCE;
  }


  @Override
  public synchronized void stop() {
      setOpenLoop(0);
  }

  @Override
  public void outputTelemetry() {
  }

  @Override
  public void zeroSensors() {
      // no-op
  }
}
