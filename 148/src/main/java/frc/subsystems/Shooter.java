package frc.subsystems;

import java.util.Optional;
import java.util.Set;

import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.team254.drivers.LazyTalonFX;

import frc.loops.ILooper;
import frc.loops.Loop;
import frc.subsystems.requests.Request;
import frc.vision.ShooterAimingParameters;
import com.team1323.lib.util.InterpolatingDouble;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.Constants;
import frc.RobotMap;
import frc.RobotState;

public class Shooter extends Subsystem {

  private LazyTalonFX ShooterMaster;
  private RobotState robotState;

  private static Shooter instance = null;
  private double mCurrentVelocitySetpoint;

  public static Shooter getInstance() {
    if (instance == null)
      instance = new Shooter();
    return instance;
  }

  private ShooterState currentState = ShooterState.OPEN_LOOP;
     public enum ShooterState {
         OFF, OPEN_LOOP, AT_GOAL, BACK_LINE, LAUNCH_PAD, HP_WALL, EXPERIMENTAL_VISION, RESET;
     }
 
     public ShooterState getState() {
         return currentState;
     }
 
     private boolean stateChanged = false;
     private double stateChangeTimestamp = Double.POSITIVE_INFINITY;
 
     public void setState(ShooterState newState) {
         if (newState != currentState) {
             currentState = newState;
             stateChanged = true;
             stateChangeTimestamp = Timer.getFPGATimestamp();
         }
     }

  private Shooter() {

    ShooterMaster = new LazyTalonFX(RobotMap.FLYWHEEL_MASTER);
    ShooterMaster.configFactoryDefault();
    ShooterMaster.setNeutralMode(NeutralMode.Coast);
    ShooterMaster.enableVoltageCompensation(true);
    ShooterMaster.configVoltageCompSaturation(10.0);
    ShooterMaster.setInverted(true);

    ShooterMaster.config_kP(0, Constants.Shooter.FLYWHEEL_KP);
    ShooterMaster.config_kI(0, Constants.Shooter.FLYWHEEL_KI);
    ShooterMaster.config_kD(0, Constants.Shooter.FLYWHEEL_KD);
    ShooterMaster.config_kF(0, Constants.Shooter.FLYWHEEL_KF);

    ShooterMaster.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_10Ms, 10);    	
    ShooterMaster.configVelocityMeasurementWindow(32, 10);

    ShooterMaster.setStatusFramePeriod(StatusFrame.Status_1_General, 50, Constants.kLongCANTimeoutMs);
    ShooterMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 50, Constants.kLongCANTimeoutMs);
    ShooterMaster.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 1000, Constants.kLongCANTimeoutMs);
    ShooterMaster.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 1000, Constants.kLongCANTimeoutMs);
    ShooterMaster.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 1000, Constants.kLongCANTimeoutMs);
    ShooterMaster.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 1000, Constants.kLongCANTimeoutMs);
    ShooterMaster.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 1000, Constants.kLongCANTimeoutMs);
    ShooterMaster.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 1000, Constants.kLongCANTimeoutMs);
    ShooterMaster.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 1000, Constants.kLongCANTimeoutMs);

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
  
  public double getSpeed() {
    double currentVelocity = ShooterMaster.getSelectedSensorVelocity();

    return currentVelocity;
  }

  public boolean hasReachedSpeed() {
    return Math.abs(getSpeed() - mCurrentVelocitySetpoint) < Constants.Shooter.SHOOTER_VELOCITY_TOLERANCE;
  }

  public void startExperimentalVision() {
    setState(ShooterState.EXPERIMENTAL_VISION);
    Optional<ShooterAimingParameters> aimRange = robotState.getExperimentalAimingParameters();
    double temp_range = aimRange.get().getRange();
    setVelocity(Constants.kVisionSpeedTreemap.getInterpolated(new InterpolatingDouble(temp_range)).value);
  }

    private final Loop loop = new Loop() {
  
    @Override
    public void onStart(double timestamp) {
      ShooterMaster.setNeutralMode(NeutralMode.Coast);
      setState(ShooterState.RESET);
    }

    @Override
    public void onLoop(double timestamp) {
        // System.out.println("Starting ONLOOP METHOD");
        switch(currentState) {
            case OPEN_LOOP:
              stop();
              break;
            case AT_GOAL:
              setVelocity(Constants.Shooter.AT_GOAL);
              break;
          case BACK_LINE:
              setVelocity(Constants.Shooter.BACK_LINE);
              break;
          case LAUNCH_PAD:
              setVelocity(Constants.Shooter.LAUNCH_PAD);
              break;
          case HP_WALL:
              setVelocity(Constants.Shooter.HP_WALL);
              break;
          case EXPERIMENTAL_VISION:
              Optional<ShooterAimingParameters> aimRange  = robotState.getExperimentalAimingParameters();
              double temp_range = aimRange.get().getRange();
              setVelocity(Constants.kVisionSpeedTreemap.getInterpolated(new InterpolatingDouble(temp_range)).value);
              break;
          case RESET:
              setVelocity(0.0);
              break;
            default:
            // System.out.println("Never matched a case!!!");
            break;
        }
    }

    @Override
    public void onStop(double timestamp) {
      ShooterMaster.setNeutralMode(NeutralMode.Coast);
    }
  };

  @Override
     public void registerEnabledLoops(ILooper enabledLooper) {
         enabledLooper.register(loop);
  }

  public Request stateRequest(ShooterState desiredState) {
    return new Request(){
    
        @Override
        public void act() {
            setState(desiredState);
        }
    };
  }

  public Request startExperimentalVisionRequest() {
    System.out.println("Start Exeperimenetal Vision Request");
    return new Request(){
        
        @Override
        public void act() {
            System.out.println("ACTING");
            startExperimentalVision();
        }
        
        @Override
        public boolean isFinished() {
            if (hasReachedSpeed()) {
                DriverStation.reportError("Hood Request Finished", false);
                return true;
            }
            return false;
        }
        
    };
  }
}
