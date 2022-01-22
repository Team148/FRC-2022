package frc.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.Constants;
import frc.RobotMap;
import frc.loops.ILooper;
import frc.loops.Loop;

public class BallHandoff extends Subsystem {
  private WPI_VictorSPX hopperMaster = new WPI_VictorSPX(RobotMap.HOPPER_MASTER);
  private WPI_TalonSRX feederMaster = new WPI_TalonSRX(RobotMap.FEEDER_MASTER);
  private static BallHandoff instance = null;

  private BallHandoff(){
    hopperMaster.configFactoryDefault();
    hopperMaster.set(ControlMode.PercentOutput, 0.0);
    hopperMaster.setNeutralMode(NeutralMode.Coast);
    hopperMaster.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
    hopperMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 500);
    hopperMaster.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 500);
    hopperMaster.setStatusFramePeriod(StatusFrame.Status_6_Misc, 500);
    hopperMaster.setStatusFramePeriod(StatusFrame.Status_7_CommStatus, 500);
    hopperMaster.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 500);
    hopperMaster.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 500);
    hopperMaster.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 500);
    hopperMaster.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 500);
    hopperMaster.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 500);
    hopperMaster.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 500);
    hopperMaster.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 500);

    feederMaster.configFactoryDefault();
    feederMaster.set(ControlMode.PercentOutput, 0.0);
    feederMaster.setNeutralMode(NeutralMode.Coast);
    feederMaster.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
    feederMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 500);
    feederMaster.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 500);
    feederMaster.setStatusFramePeriod(StatusFrame.Status_6_Misc, 500);
    feederMaster.setStatusFramePeriod(StatusFrame.Status_7_CommStatus, 500);
    feederMaster.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 500);
    feederMaster.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 500);
    feederMaster.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 500);
    feederMaster.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 500);
    feederMaster.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 500);
    feederMaster.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 500);
    feederMaster.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 500);
    feederMaster.setInverted(false);
  }

  public static BallHandoff getInstance() {
      if(instance == null)
        instance = new BallHandoff();
      return instance;
  }

  public void setMotors(double feeder, double hopper) {
    hopperMaster.set(hopper);
    feederMaster.set(feeder);
  }

  synchronized void setOpenLoop(double feeder, double hopper) {
    hopperMaster.set(ControlMode.PercentOutput, hopper);
    feederMaster.set(ControlMode.PercentOutput, feeder);
  }

  @Override
  public synchronized void stop() {
      setOpenLoop(0,0);
  }

  @Override
  public void outputTelemetry() {
  }

  @Override
  public void zeroSensors() {
  }

  public enum BallHandoffState {
      OFF,
      INTAKING,
      SHOOTING,
      UNJAM_FEED,
      UNJAM_HOPPER,
      WHEEL_OF_FORTUNE,
      POOP;
  }

  private BallHandoffState currentState = BallHandoffState.OFF;
  private boolean stateChanged = false;

  public BallHandoffState getState() {
      return currentState;
    }

  public synchronized void setState(BallHandoffState newState) {
      if (newState != currentState)
          stateChanged = true;
  currentState = newState;
  }

  private final Loop loop = new Loop() {

  @Override
  public void onStart(double timestamp) {
    setState(BallHandoffState.OFF);
    stop();
  }

  @Override
  public void onLoop(double timestamp) {
      switch (currentState) {
      case OFF:
          stop();
          break;
      case INTAKING:
          setMotors(0.0, Constants.BallHandoff.HOPPER_INTAKING_SPEED + 0.2);
          break;
      case SHOOTING:
          // if(Math.abs(Robot.getHood().getAngle().getUnboundedDegrees() - Robot.getHood().getSetpoint()) < Constants.HOOD_SHOOTING_DEGREE_TOLERANCE){
            // if(Robot.getShooter().atTargetVelocity()){
              // TODO maybe check if vision on target
              setMotors(Constants.BallHandoff.FEEDER_SHOOTING_SPEED, Constants.BallHandoff.HOPPER_SHOOTING_SPEED + 0.05);
            // }
          // }
          break;
      case UNJAM_FEED:
          setMotors(Constants.BallHandoff.FEEDER_UNJAM_SPEED, 0.0);
          break;
      case UNJAM_HOPPER:
          setMotors(0.0, Constants.BallHandoff.HOPPER_UNJAM_SPEED);
          break;
      case WHEEL_OF_FORTUNE:
          setMotors(0.0, Constants.BallHandoff.HOPPER_WOF_SPEED);
          break;
      case POOP:
          setMotors(0.38, -0.35);
      default:
          break;

    }
    if (stateChanged)
      stateChanged = false;
  }

  @Override
  public void onStop(double timestamp) {
    setState(BallHandoffState.OFF);
    stop();
  }

  };
  @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
      enabledLooper.register(loop);
    }
}
