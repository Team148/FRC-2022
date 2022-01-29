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

public class IntakePivot extends Subsystem {

    // Motor and Sensors Instantiation
    private LazyTalonFX intakePivot;
    private RobotState robotState;

    PeriodicIO periodicIO = new PeriodicIO();

    // Class Specific Updatable Variables
    public double targetAngle = 0.0;
    private double stateEnteredTimestamp = 0.0;
    private double lastTrackWhileShooting = Double.POSITIVE_INFINITY;
    private boolean isEncoderFlipped = false;
    private boolean zeroedAbsolutely = false;

    public double getTargetAngle() {
        return targetAngle;
    }

    private static IntakePivot instance = null;
    public static IntakePivot getInstance() {
        if (instance == null)
            instance = new IntakePivot();
        return instance;
    }

    private State currentState = State.OPEN_LOOP;
    public enum State {
        OPEN_LOOP, POSITION, AWAITING_LOCK;
    }

    public State getState() {
        return currentState;
    }

    private boolean stateChanged = false;
    private double stateChangeTimestamp = Double.POSITIVE_INFINITY;

    private boolean aimInnerPort = false;

    public void setState(State newState) {
        if (newState != currentState) {
            currentState = newState;
            stateChanged = true;
            stateChangeTimestamp = Timer.getFPGATimestamp();
        }
    }

    public IntakePivot() {

        // Setting instantiated motors and sensors
        intakePivot = new LazyTalonFX(RobotMap.INTAKE_PIVOT_MASTER);
        intakePivot.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);// encoder = new DutyCycle(new DigitalInput(Ports.IntakePivot_ENCODER));
        robotState = RobotState.getInstance();

        // Config TalonFX
        intakePivot.configVoltageCompSaturation(12.0, Constants.kCANTimeoutMs);
        intakePivot.enableVoltageCompensation(true);
        intakePivot.setInverted(TalonFXInvertType.Clockwise);        
        intakePivot.configNominalOutputForward(0.0 / 12.0, Constants.kCANTimeoutMs);

        SupplyCurrentLimitConfiguration currentLimitConfiguration = new SupplyCurrentLimitConfiguration(true, 25, 30, Constants.kCANTimeoutMs);
        intakePivot.configSupplyCurrentLimit(currentLimitConfiguration, Constants.kCANTimeoutMs);

        intakePivot.selectProfileSlot(0, 0);
        intakePivot.config_kP(0, Constants.IntakePivot.INTAKEPIVOT_KP, Constants.kCANTimeoutMs);
        intakePivot.config_kI(0, Constants.IntakePivot.INTAKEPIVOT_KI, Constants.kCANTimeoutMs);
        intakePivot.config_kD(0, Constants.IntakePivot.INTAKEPIVOT_KD, Constants.kCANTimeoutMs);
        intakePivot.config_kF(0, Constants.IntakePivot.INTAKEPIVOT_KF, Constants.kCANTimeoutMs);

        intakePivot.configMotionCruiseVelocity((int)(Constants.IntakePivot.INTAKEPIVOT_MAXSPEED), Constants.kCANTimeoutMs);
        intakePivot.configMotionAcceleration((int)(Constants.IntakePivot.INTAKEPIVOT_MAXSPEED * 3.0), Constants.kCANTimeoutMs);
        intakePivot.configMotionSCurveStrength(0);

        intakePivot.configForwardSoftLimitThreshold(intakePivotDegreesToInternalEncUnits(Constants.IntakePivot.INTAKEPIVOT_MAXCONTROLANGLE), Constants.kCANTimeoutMs);
        intakePivot.configReverseSoftLimitThreshold(intakePivotDegreesToInternalEncUnits(Constants.IntakePivot.INTAKEPIVOT_MINCONTROLANGLE), Constants.kCANTimeoutMs);
        intakePivot.configForwardSoftLimitEnable(true, Constants.kCANTimeoutMs);
        intakePivot.configReverseSoftLimitEnable(true, Constants.kCANTimeoutMs);

        System.out.println("intakePivot max soft limit: " + intakePivotDegreesToInternalEncUnits(Constants.IntakePivot.INTAKEPIVOT_MAXCONTROLANGLE));
        System.out.println("intakePivot min soft limit: " + intakePivotDegreesToInternalEncUnits(Constants.IntakePivot.INTAKEPIVOT_MINCONTROLANGLE));

        intakePivot.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
        intakePivot.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 50);

        //setOpenLoop(0.0);

        setEncoderPhase(true);

    }

    private void setEncoderPhase(boolean phase) {
        isEncoderFlipped = phase;
    }

    // private double getAbsoluteEncoderDegrees() {
    //     return (isEncoderFlipped ? -1.0 : 1.0) * encoder.getOutput() * 360.0;
    // }

    // private boolean isEncoderConnected() {
    //     if (RobotBase.isReal()) {
    //         //System.out.println("Robot Base: " + RobotBase.isReal() + " Encoder Freq: " + encoder.getFrequency());
    //         return (encoder.getFrequency() != 0) ? true : false;
    //     }
    //     return true;
    // }

    private void setAngle(double angle) {
        targetAngle = boundTointakePivotRange(angle);
        /*if (ActuatingHood.getInstance().isStowed())
            targetAngle = closestPole();
            */
        periodicIO.controlMode = ControlMode.MotionMagic;
        periodicIO.demand = intakePivotDegreesToInternalEncUnits(targetAngle);
        // System.out.println("Demand is: " + periodicIO.demand);
    }

    private boolean inRange(double value, double min, double max) {
        return min <= value && value <= max;
    }


    public boolean inintakePivotRange(double angle) {
        angle = Util.boundAngle0to360Degrees(angle);
        if (!inRange(angle, Constants.IntakePivot.INTAKEPIVOT_MINCONTROLANGLE, Constants.IntakePivot.INTAKEPIVOT_MAXCONTROLANGLE)) {
            angle = (angle < Constants.IntakePivot.INTAKEPIVOT_MINCONTROLANGLE) ? (angle + 360.0) : (angle - 360.0);
            return inRange(angle, Constants.IntakePivot.INTAKEPIVOT_MINCONTROLANGLE, Constants.IntakePivot.INTAKEPIVOT_MAXCONTROLANGLE);
        }
        return true;
    }

    /** 
     * Takes an angle from any scope and finds the equivalent angle in the intakePivot's range of motion or,
     * if one doesn't exist, returns the intakePivot soft limit closest to the desired angle.
    */
    public double boundTointakePivotRange(double angle) {
        angle = Util.placeInAppropriate0To360Scope(getAngle(), angle);

        if (!inRange(angle, Constants.IntakePivot.INTAKEPIVOT_MINCONTROLANGLE, Constants.IntakePivot.INTAKEPIVOT_MAXCONTROLANGLE)) {
            angle = (angle < Constants.IntakePivot.INTAKEPIVOT_MINCONTROLANGLE) ? (angle + 360.0) : (angle - 360.0);
            if (inRange(angle, Constants.IntakePivot.INTAKEPIVOT_MINCONTROLANGLE, Constants.IntakePivot.INTAKEPIVOT_MAXCONTROLANGLE))
                return angle;
            return (Math.abs(Rotation2d.fromDegrees(angle).distance(Rotation2d.fromDegrees(Constants.IntakePivot.INTAKEPIVOT_MINCONTROLANGLE)))
                < Math.abs(Rotation2d.fromDegrees(angle).distance(Rotation2d.fromDegrees(Constants.IntakePivot.INTAKEPIVOT_MAXCONTROLANGLE))))
                ? Constants.IntakePivot.INTAKEPIVOT_MINCONTROLANGLE : Constants.IntakePivot.INTAKEPIVOT_MAXCONTROLANGLE;
        }

        return angle;
    }

    public void setPosition(double angle) {
        setState(State.POSITION);
        setAngle(angle);
    }

    public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(getAngle());
    }

    public void lockAngle() {
        setState(State.AWAITING_LOCK);
    }

    public void setOpenLoop(double output) {
        periodicIO.controlMode = ControlMode.PercentOutput;
        periodicIO.demand = /*Hood.getInstance().isStowed() ? 0.0 : */Util.scaledDeadband(output, 1.0, 0.25) * 0.5;
        setState(State.OPEN_LOOP);
    }
    
    public boolean isOpenLoop() {
        return currentState == State.OPEN_LOOP;
    }
    
    public boolean hasReachedAngle() {
        return Math.abs(getAngle() - targetAngle) < Constants.IntakePivot.kAngleTolerance;
    }

    public double getAngle() {
        return internalEncUnitsToIntakePivotDegrees(periodicIO.position);
    }

    @Override
    public synchronized void readPeriodicInputs() {
        periodicIO.position = (int)intakePivot.getSelectedSensorPosition(0);
        if (Settings.debugIntakePivot()) {
            periodicIO.velocity = (int)intakePivot.getSelectedSensorVelocity(0);
            periodicIO.voltage = intakePivot.getMotorOutputVoltage();
            periodicIO.current = intakePivot.getOutputCurrent();
        }
    }
    
    @Override
    public void writePeriodicOutputs() {
        if (zeroedAbsolutely) {
            intakePivot.set(periodicIO.controlMode, periodicIO.demand);
        }
    }

    private final Loop loop = new Loop() {

        @Override
        public void onStart(double timestamp) {
            intakePivot.setNeutralMode(NeutralMode.Brake);
        }

        @Override
        public void onLoop(double timestamp) {
            // System.out.println("Starting ONLOOP METHOD");
            switch(currentState) {
                case POSITION:
                  break;
                case AWAITING_LOCK:
                    if (stateChanged) {
                        // System.out.println("Current state is OPENLOOP");
                        periodicIO.controlMode = ControlMode.PercentOutput;
                        periodicIO.demand = 0.0;
                    }
                    // System.out.println("Current state is POSITION");
                    if (timestamp - stateChangeTimestamp > 0.25)
                        setPosition(getAngle());
                default:
                // System.out.println("Never matched a case!!!");
                break;
            }
        }

        @Override
        public void onStop(double timestamp) {
            intakePivot.setNeutralMode(NeutralMode.Coast);
        }
    };

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(loop);
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("intakePivot Angle", getAngle());
        //SmartDashboard.putNumber("intakePivot Current", periodicIO.current);
        //SmartDashboard.putNumber("intakePivot Voltage", periodicIO.voltage);
        //SmartDashboard.putBoolean("intakePivot Zeroed Absolutely", zeroedAbsolutely);
        // SmartDashboard.putNumber("intakePivot Absolute Angle", getAbsoluteEncoderDegrees());
        if (Settings.debugIntakePivot()) {
            SmartDashboard.putString("intakePivot State", currentState.toString());
            // SmartDashboard.putNumber("intakePivot Absolute Frequency", encoder.getFrequency());
            SmartDashboard.putNumber("intakePivot Encoder", periodicIO.position);
            SmartDashboard.putNumber("intakePivot Velocity", periodicIO.velocity);
            SmartDashboard.putNumber("intakePivot Angle Error", internalEncUnitsToIntakePivotDegrees(intakePivot.getClosedLoopError(0)));
            if (intakePivot.getControlMode() == ControlMode.MotionMagic) {
                SmartDashboard.putNumber("intakePivot Setpoint", intakePivot.getClosedLoopTarget(0));
            }
        }
    }

    @Override
    public void stop() {
        setOpenLoop(0.0);
    }

    public Request angleRequest(double angle, double speedScalar, boolean waitForAngle) {
        return new Request(){
            
            @Override
            public void act() {
                intakePivot.configMotionCruiseVelocity((int)(Constants.IntakePivot.INTAKEPIVOT_MAXSPEED * speedScalar), Constants.kCANTimeoutMs);
                setPosition(angle);
            }
            
            @Override
            public boolean isFinished() {
                if (waitForAngle) {
                    return hasReachedAngle();
                } else {
                    return true;
                }
            }
            
        };
    }
    
    public Request angleRequest(double angle) {
        return angleRequest(angle, 1.0, true);
    }
    
    public Request angleRequest(double angle, double speedScalar) {
        return angleRequest(angle, speedScalar, true);
    }

    public Request safeLockAngleRequest() {
        return new Request(){
        
            @Override
            public void act() {
                setOpenLoop(0.0);
            }
        };
    }
    
    public Request lockAngleRequest() {
        return new Request(){
            
            @Override
            public void act() {
                lockAngle();
            }
        };
    }

    public Request stateRequest(State desiredState) {
        return new Request(){
        
            @Override
            public void act() {
                setState(desiredState);
            }
        };
    }

    public int intakePivotDegreesToInternalEncUnits(double intakePivotAngle) {
        return (int) ((intakePivotAngle / 360.0) * Constants.IntakePivot.kInternalEncToOutputRatio * 2048.0);
    }

    public double internalEncUnitsToIntakePivotDegrees(double encUnits) {
        return (encUnits / 2048.0) / Constants.IntakePivot.kInternalEncToOutputRatio * 360.0;
    }

    public synchronized void resetToAbsolute() {
        if (!zeroedAbsolutely) {
            //System.out.println("intakePivot Encoder Connected: " + isEncoderConnected());
            if (RobotBase.isReal()) {
                //DriverStation.reportError("intakePivot WAS RESET TO ABSOLUTE WITH THE MAG ENCODER", false);
                double absolutePosition = Util.boundAngle0to360Degrees(90.0);//getAbsoluteEncoderDegrees() - Constants.intakePivot.kEncoderStartingAngle);
                if (absolutePosition > Constants.IntakePivot.INTAKEPIVOT_MAXINITIALANGLE)
                    absolutePosition -= 360.0;
                else if (absolutePosition < Constants.IntakePivot.INTAKEPIVOT_MININITIALLANGLE)
                    absolutePosition += 360.0;
                if (!inRange(absolutePosition, Constants.IntakePivot.INTAKEPIVOT_MININITIALLANGLE, Constants.IntakePivot.INTAKEPIVOT_MAXINITIALANGLE)) {
                    DriverStation.reportError("intakePivot angle is out of bounds", false);
                    hasEmergency = true;
                }
                intakePivot.setSelectedSensorPosition(intakePivotDegreesToInternalEncUnits(absolutePosition));
                // intakePivot.setSelectedSensorPosition(intakePivotDegreesToInternalEncUnits(absolutePosition), 0, Constants.kCANTimeoutMs);
                //System.out.println("intakePivot Absolute angle: " + getAbsoluteEncoderDegrees() + ", encoder offset: " + Constants.intakePivot.kEncoderStartingAngle + ", difference: " + (getAbsoluteEncoderDegrees() - Constants.intakePivot.kEncoderStartingAngle) + ", degreesToEncUnits: " + intakePivotDegreesToInternalEncUnits(getAbsoluteEncoderDegrees() - Constants.intakePivot.kEncoderStartingAngle));
            } else {
                DriverStation.reportError("intakePivot encoder NOT DETECTED: CURRENT POSITION SET TO 0", false);
                intakePivot.setSelectedSensorPosition(intakePivotDegreesToInternalEncUnits(90.0));
                // intakePivot.setSelectedSensorPosition(0, 0, Constants.kCANTimeoutMs);
            }
        }
    }

    public synchronized boolean getCanFold() {
        return inRange(getAngle(), -15.0, 15.0) || inRange(getAngle(), 165.0, 195.0);
    }

    public double closestPole() {
        return (Math.abs(getAngle() - 180.0) < Math.abs(getAngle() - 0.0)) ? 180.0 : 0.0;
    }

    public void setNearestFoldAngle() {
        setPosition(closestPole());
    }

    public boolean isGoingToPole() {
        return Util.epsilonEquals(targetAngle, 0.0) || Util.epsilonEquals(targetAngle, 180.0);
    }

    public synchronized void zeroedintakePivot() {
        System.out.println("intakePivot Zeroed");
        System.out.println("intakePivot Absolute angle: 0.0" + ", encoder offset: " + Constants.IntakePivot.kEncoderStartingAngle + ", difference: 0.0");
        zeroedAbsolutely = true;
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