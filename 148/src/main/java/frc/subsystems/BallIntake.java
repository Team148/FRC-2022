package frc.subsystems;

import frc.Constants;
import frc.RobotMap;
import frc.loops.ILooper;
import frc.loops.Loop;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;

import com.team254.drivers.LazyTalonFX;

public class BallIntake extends Subsystem {
  private LazyTalonFX ballIntakeMaster = new LazyTalonFX(RobotMap.INTAKE_MASTER);
  private static BallIntake instance = null;

  private BallIntake(){
    ballIntakeMaster.configFactoryDefault();
    ballIntakeMaster.set(ControlMode.PercentOutput, 0.0);
    ballIntakeMaster.setNeutralMode(NeutralMode.Coast);
    ballIntakeMaster.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
    ballIntakeMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 500);
    ballIntakeMaster.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 500);
    ballIntakeMaster.setStatusFramePeriod(StatusFrame.Status_6_Misc, 500);
    ballIntakeMaster.setStatusFramePeriod(StatusFrame.Status_7_CommStatus, 500);
    ballIntakeMaster.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 500);
    ballIntakeMaster.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 500);
    ballIntakeMaster.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 500);
    ballIntakeMaster.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 500);
    ballIntakeMaster.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 500);
    ballIntakeMaster.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 500);
    ballIntakeMaster.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 500);
    ballIntakeMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 60, 100, 1.5));
    ballIntakeMaster.setInverted(false);
  }

  public static BallIntake getInstance() {
      if(instance == null)
        instance = new BallIntake();
      return instance;
  }

  public void setMotor(double speed) {
    ballIntakeMaster.set(ControlMode.PercentOutput, speed);
  }

  synchronized void setOpenLoop(double speed) {
    ballIntakeMaster.set(ControlMode.PercentOutput, speed);
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

  private BallIntakeState currentState = BallIntakeState.OFF;
  private boolean stateChanged = false;

  public BallIntakeState getState() {
      return currentState;
    }

  public synchronized void setState(BallIntakeState newState) {
      if (newState != currentState)
          stateChanged = true;
  currentState = newState;
  }

  private final Loop loop = new Loop() {

    @Override
    public void onStart(double timestamp) {
      setState(BallIntakeState.OFF);
      stop();
    }
  
    @Override
    public void onLoop(double timestamp) {
        switch (currentState) {
        case OFF:
            stop();
            break;
        case INTAKING:
            setMotor(Constants.BallIntake.INTAKING_SPEED);
            break;
        case OUTTAKING:
            setMotor(Constants.BallIntake.OUTTAKING_SPEED);
        default:
            break;
  
      }
      if (stateChanged)
        stateChanged = false;
    }
  
    @Override
    public void onStop(double timestamp) {
      setState(BallIntakeState.OFF);
      stop();
    }
  
    };
    @Override
      public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(loop);
      }
  
    public enum BallIntakeState {
      OFF,
      INTAKING,
      OUTTAKING
    }
}
