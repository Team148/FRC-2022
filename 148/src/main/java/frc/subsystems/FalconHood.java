package frc.subsystems;

import java.util.Optional;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import frc.Constants;
import frc.RobotMap;
import frc.RobotState;
import frc.Settings;
import frc.loops.ILooper;
import frc.loops.Loop;
import frc.subsystems.requests.Request;
import frc.vision.ShooterAimingParameters;
import com.team1323.lib.util.Util;
import com.team254.drivers.LazyTalonFX;
import com.team254.lib.geometry.Rotation2d;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class FalconHood extends Subsystem{

     // Motor and Sensors Instantiation
     private LazyTalonFX hoodFalcon;
 
     PeriodicIO periodicIO = new PeriodicIO();
 
     // Class Specific Updatable Variables
     public double targetAngle = 0.0;
     private double stateEnteredTimestamp = 0.0;
     private double lastTrackWhileShooting = Double.POSITIVE_INFINITY;
     private boolean isEncoderFlipped = false;
     private boolean zeroedAbsolutely = true;
 
     public double getTargetAngle() {
         return targetAngle;
     }
 
     private static FalconHood instance = null;
     public static FalconHood getInstance() {
         if (instance == null)
             instance = new FalconHood();
         return instance;
     }
 
     private HoodState currentState = HoodState.OPEN_LOOP;
     public enum HoodState {
         OPEN_LOOP, POSITION;
     }
 
     public HoodState getState() {
         return currentState;
     }
 
     private boolean stateChanged = false;
     private double stateChangeTimestamp = Double.POSITIVE_INFINITY;
 
     public void setState(HoodState newState) {
         if (newState != currentState) {
             currentState = newState;
             stateChanged = true;
             stateChangeTimestamp = Timer.getFPGATimestamp();
         }
     }
 
     public FalconHood() {
 
         // Setting instantiated motors and sensors
         hoodFalcon = new LazyTalonFX(RobotMap.HOOD_MASTER);
         hoodFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
 
         // Config TalonFX
         hoodFalcon.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
         hoodFalcon.enableVoltageCompensation(true);
         hoodFalcon.setInverted(TalonFXInvertType.CounterClockwise);        
         hoodFalcon.configNominalOutputForward(0.0 / 12.0, Constants.kLongCANTimeoutMs);
 
         SupplyCurrentLimitConfiguration currentLimitConfiguration = new SupplyCurrentLimitConfiguration(true, 25, 30, Constants.kLongCANTimeoutMs);
         hoodFalcon.configSupplyCurrentLimit(currentLimitConfiguration, Constants.kLongCANTimeoutMs);
 
         hoodFalcon.selectProfileSlot(0, 0);
         hoodFalcon.config_kP(0, Constants.FalconHood.kP, Constants.kLongCANTimeoutMs);
         hoodFalcon.config_kI(0, Constants.FalconHood.kI, Constants.kLongCANTimeoutMs);
         hoodFalcon.config_kD(0, Constants.FalconHood.kD, Constants.kLongCANTimeoutMs);
 
        //  hoodFalcon.configForwardSoftLimitEnable(true, Constants.kCANTimeoutMs);
        //  hoodFalcon.configReverseSoftLimitEnable(true, Constants.kCANTimeoutMs);
 
        
         hoodFalcon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20, Constants.kLongCANTimeoutMs);
         hoodFalcon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 50, Constants.kLongCANTimeoutMs);
 
         //setOpenLoop(0.0);
 
         setEncoderPhase(true);
 
     }
    //  private double getAbsoluteEncoderDegrees() {
    //     //     // return (isEncoderFlipped ? -1.0 : 1.0) * encoder.getOutput() * 360.0;
    //     //     return (isEncoderFlipped ? -1.0 : 1.0) * encoder.getAbsolutePosition();
    //     double aNumber = 0.0;
    //     return aNumber;
    //  }
     
    
     public int degreesToEncUnits(double degrees) {
         return (int)(degrees * 0.0);
     }

     public double encUnitsToDegrees(double encUnits) {
         return encUnits / 0.0;
     }
 
     private void setEncoderPhase(boolean phase) {
         isEncoderFlipped = phase;
     }
 
     public void setHoodPosition(double angle) {

        setState(HoodState.POSITION);
         if (angle < Constants.FalconHood.kMinControlAngle) angle = Constants.FalconHood.kMinControlAngle;
         if (angle > Constants.FalconHood.kMaxControlAngle) angle = Constants.FalconHood.kMaxControlAngle;

        //  hoodFalcon.set(ControlMode.Position, setpoint);
        // System.out.println("Setting hood position");

        int setpoint = angleToEncoderUnits(angle);
        periodicIO.controlMode = ControlMode.Position;
        periodicIO.demand = setpoint;

        writePeriodicOutputs();
     }
 
     public int angleToEncoderUnits(double angle) {
        double setpoint = (angle / 360.0) * (Constants.FalconHood.kEncoderRatio * 2048.0);
        // System.out.println("Setting position to " + setpoint);
        return (int) setpoint;
     }
 
     public void setOpenLoop(double output) {
         // periodicIO.controlMode = ControlMode.PercentOutput;
         double demand = /*Hood.getInstance().isStowed() ? 0.0 : */Util.scaledDeadband(output, 1.0, 0.25) * 0.5;
         hoodFalcon.set(ControlMode.PercentOutput, demand);
     }
     
     public boolean isOpenLoop() {
        setState(HoodState.OPEN_LOOP);
         return currentState == HoodState.OPEN_LOOP;
     }

     public synchronized void zeroSensors() {
         hoodFalcon.setSelectedSensorPosition(angleToEncoderUnits(Constants.FalconHood.kMinControlAngle));
     }
 
     @Override
     public synchronized void readPeriodicInputs() {
         periodicIO.position = (int)hoodFalcon.getSelectedSensorPosition(0);
         if (Settings.debugFalconHood()) {
             periodicIO.velocity = (int)hoodFalcon.getSelectedSensorVelocity(0);
             periodicIO.voltage = hoodFalcon.getMotorOutputVoltage();
             periodicIO.current = hoodFalcon.getOutputCurrent();
         }
     }
     
     
     @Override
     public void writePeriodicOutputs() {
            hoodFalcon.set(periodicIO.controlMode, periodicIO.demand);
     }
 
     private final Loop loop = new Loop() {
 
         @Override
         public void onStart(double timestamp) {
            hoodFalcon.setNeutralMode(NeutralMode.Brake);
         }
 
         @Override
         public void onLoop(double timestamp) {
             // System.out.println("Starting ONLOOP METHOD");
             switch(currentState) {
                 case OPEN_LOOP:
                    break;
                 case POSITION:

                    break;
                 default:
                 // System.out.println("Never matched a case!!!");
                 break;
             }
         }
 
         @Override
         public void onStop(double timestamp) {
            hoodFalcon.setNeutralMode(NeutralMode.Coast);
         }
     };
 
     @Override
     public void registerEnabledLoops(ILooper enabledLooper) {
         enabledLooper.register(loop);
     }
 
     @Override
     public void outputTelemetry() {
     }
 
     @Override
     public void stop() {
         setOpenLoop(0.0);
     }
 
     public Request stateRequest(HoodState desiredState) {
         return new Request(){
         
             @Override
             public void act() {
                 setState(desiredState);
             }
         };
     }
 
     public synchronized void resetToAbsolute() {
     }
 
     private static class PeriodicIO {
         //Inputs
         public int position;
         public int velocity;
         public double voltage;
         public double current;
         
         //Outputs
         public double demand = 0.0;
         public ControlMode controlMode = ControlMode.PercentOutput;
     }
}
