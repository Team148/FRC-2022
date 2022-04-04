package frc.subsystems;

import frc.Constants;
import frc.RobotMap;
import frc.loops.ILooper;
import frc.loops.Loop;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

// import edu.wpi.first.wpilibj.Servo;
import com.team1323.lib.util.LinearServo;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

public class Hanger extends Subsystem {
  private WPI_TalonFX hangerMaster = new WPI_TalonFX(RobotMap.HANGER_MASTER);
  private WPI_TalonFX hangerFollower = new WPI_TalonFX(RobotMap.HANGER_FOLLOWER);

  private LinearServo oneClawServo;
  private LinearServo twoClawServo;

  private static Hanger instance = null;

  private Hanger(){
    hangerMaster.configFactoryDefault();
    hangerMaster.set(ControlMode.PercentOutput, 0.0);
    hangerMaster.setInverted(true);
    hangerMaster.setNeutralMode(NeutralMode.Brake);
    hangerMaster.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
    hangerMaster.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 500);
    hangerMaster.setStatusFramePeriod(StatusFrame.Status_6_Misc, 500);
    hangerMaster.setStatusFramePeriod(StatusFrame.Status_7_CommStatus, 500);
    hangerMaster.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 500);
    hangerMaster.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 500);
    hangerMaster.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 500);
    hangerMaster.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 500);
    hangerMaster.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 500);
    hangerMaster.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 500);
    hangerMaster.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 500);

    hangerFollower.configFactoryDefault();
    hangerFollower.set(ControlMode.PercentOutput, 0.0);
    hangerFollower.setInverted(true);
    hangerFollower.setNeutralMode(NeutralMode.Brake);
    hangerFollower.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
    hangerFollower.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 500);
    hangerFollower.setStatusFramePeriod(StatusFrame.Status_6_Misc, 500);
    hangerFollower.setStatusFramePeriod(StatusFrame.Status_7_CommStatus, 500);
    hangerFollower.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 500);
    hangerFollower.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 500);
    hangerFollower.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 500);
    hangerFollower.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 500);
    hangerFollower.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 500);
    hangerFollower.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 500);
    hangerFollower.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 500);
    hangerFollower.follow(hangerMaster);
    // hangerMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 55, 65, 1.5));

    oneClawServo = new LinearServo(RobotMap.ONE_CLAW_SERVO, 100, 24);
    twoClawServo = new LinearServo(RobotMap.TWO_CLAW_SERVO, 100, 24);
  }

  public static Hanger getInstance() {
      if(instance == null)
        instance = new Hanger();
      return instance;
  }

  public void setMotor(double speed) {
    hangerMaster.set(ControlMode.PercentOutput, speed);
  }

  public void setOpenLoop(double speed) {
    hangerMaster.set(ControlMode.PercentOutput, speed);
  }

  public double getHangerPosition() {
    double position = hangerMaster.getSelectedSensorPosition();
    return position;
  }

  public boolean getHangerLimitSwitch() {
    int atLimit = hangerMaster.isRevLimitSwitchClosed();

    if(atLimit == 1) {
      return true;
    }
    else
      return false;
  }

  public void setOneClawServo(double position) {
    oneClawServo.set(position);
  }

  public void setTwoClawServo(double position) {
    twoClawServo.set(position);
  }

  public void setBothClawServos(double positionOneClaw, double positionTwoClaw) {
    oneClawServo.set(positionOneClaw);
    twoClawServo.set(positionTwoClaw);
  }

  public double getOneClawServoPosition() {
    return oneClawServo.getPosition();
  }

  public double getTwoClawServoPosition() {
    return twoClawServo.getPosition();
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

  public enum HangerState {
    OFF,
    STOW,
    EXTEND;
}

private HangerState currentState = HangerState.OFF;
private boolean stateChanged = false;

public HangerState getState() {
    return currentState;
  }

public synchronized void setState(HangerState newState) {
    if (newState != currentState)
        stateChanged = true;
currentState = newState;
}

private final Loop loop = new Loop() {

@Override
public void onStart(double timestamp) {
  setState(HangerState.OFF);
  stop();
}

@Override
public void onLoop(double timestamp) {
    switch (currentState) {
    case OFF:
        break;
    case STOW:
        if(getHangerPosition() < Constants.Hanger.HANGER_DOWN_SLOW)
            setMotor(-1.0);
        else
            setMotor(-1.0);
        break;
    case EXTEND:
        setMotor(1.0);
        break;
    default:
        break;
  }
  if (stateChanged)
    stateChanged = false;
}

@Override
public void onStop(double timestamp) {
  setState(HangerState.OFF);
  stop();
}

};
@Override
  public void registerEnabledLoops(ILooper enabledLooper) {
    enabledLooper.register(loop);
  }
}
