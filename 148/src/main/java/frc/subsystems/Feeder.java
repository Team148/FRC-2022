package frc.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.Constants;
import frc.RobotMap;
import frc.loops.ILooper;
import frc.loops.Loop;

public class Feeder extends Subsystem {
  private WPI_TalonSRX feederMaster = new WPI_TalonSRX(RobotMap.FEEDER_MASTER);
  private static Feeder instance = null;

  private Feeder(){

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

    feederMaster.overrideLimitSwitchesEnable(false);
  }

  public static Feeder getInstance() {
      if(instance == null)
        instance = new Feeder();
      return instance;
  }

  public void setMotors(double feeder) {
    feederMaster.set(feeder);
  }

  synchronized void setOpenLoop(double feeder) {
    feederMaster.set(ControlMode.PercentOutput, feeder);
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
  }

  private FeederState currentState = FeederState.OFF;
  private boolean stateChanged = false;

  public FeederState getState() {
      return currentState;
    }

  public synchronized void setState(FeederState newState) {
      if (newState != currentState)
          stateChanged = true;
  currentState = newState;
  }

  private final Loop loop = new Loop() {

  @Override
  public void onStart(double timestamp) {
    setState(FeederState.OFF);
    stop();
  }

  @Override
  public void onLoop(double timestamp) {
      switch (currentState) {
      case OFF:
          stop();
          break;
      case INTAKING:
          if (feederMaster.getSensorCollection().isRevLimitSwitchClosed()) {
            setMotors(0.0);
          }
          else {
            setMotors(Constants.Feeder.FEEDER_INTAKE_SPEED);
          }
          // setMotors(0.0, Constants.Feeder.HOPPER_INTAKING_SPEED + 0.2);
          break;
      case SHOOTING:
          setMotors(Constants.Feeder.FEEDER_SHOOTING_SPEED);
          break;
      case UNJAM_FEED:
          setMotors(Constants.Feeder.FEEDER_UNJAM_SPEED);
          break;
      case UNJAM_HOPPER:
          // setMotors(0.0, Constants.Feeder.HOPPER_UNJAM_SPEED);
          break;
      case WHEEL_OF_FORTUNE:
          // setMotors(0.0, Constants.Feeder.HOPPER_WOF_SPEED);
          break;
      case POOP:
          // setMotors(0.38, -0.35);
      default:
          break;

    }
    if (stateChanged)
      stateChanged = false;
  }

  @Override
  public void onStop(double timestamp) {
    setState(FeederState.OFF);
    stop();
  }

  };
  @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
      enabledLooper.register(loop);
    }

  public enum FeederState {
    OFF,
    INTAKING,
    SHOOTING,
    UNJAM_FEED,
    UNJAM_HOPPER,
    WHEEL_OF_FORTUNE,
    POOP;
  }
}
