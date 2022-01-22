package frc.subsystems;

import frc.RobotMap;

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
    ballIntakeMaster.setInverted(true);
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
}
