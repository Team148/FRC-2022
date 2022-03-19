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

/**
 * The turret subsystem controls the direction of the balls being fired. On the turret assembly is the Shooter.
 * The turret can only rotate within 330 degrees but soft limited to 325, the encoder is used only as a backup encoder and to reset
 * the internal encoder of the falcon. This is part of the Superstructure class.
 */
public class Turret extends Subsystem {

    // Motor and Sensors Instantiation
    private LazyTalonFX turret;
    private RobotState robotState;

    PeriodicIO periodicIO = new PeriodicIO();

    // Class Specific Updatable Variables
    public double targetAngle = 0.0;
    private double gyroLockedHeading = 0.0;
    private double gyroLockedTurretHeading = 0.0;
    private double stateEnteredTimestamp = 0.0;
    private double lastTrackWhileShooting = Double.POSITIVE_INFINITY;
    private boolean isEncoderFlipped = false;
    private boolean zeroedAbsolutely = false;

    public double getTargetAngle() {
        return targetAngle;
    }

    private static Turret instance = null;
    public static Turret getInstance() {
        if (instance == null)
            instance = new Turret();
        return instance;
    }

    private State currentState = State.OPEN_LOOP;
    public enum State {
        OPEN_LOOP, EXPERIMENTAL_VISION, VISION, POSITION, GYRO_COMP, AWAITING_LOCK;
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

    public Turret() {

        // Setting instantiated motors and sensors
        turret = new LazyTalonFX(RobotMap.TURRET_MASTER);
        turret.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);// encoder = new DutyCycle(new DigitalInput(Ports.TURRET_ENCODER));
        robotState = RobotState.getInstance();

        // Config TalonFX
        turret.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        turret.enableVoltageCompensation(true);
        turret.setInverted(TalonFXInvertType.Clockwise);        
        // turret.configNominalOutputForward(0.0 / 12.0, Constants.kLongCANTimeoutMs);

        SupplyCurrentLimitConfiguration currentLimitConfiguration = new SupplyCurrentLimitConfiguration(true, 25, 30, Constants.kLongCANTimeoutMs);
        turret.configSupplyCurrentLimit(currentLimitConfiguration, Constants.kLongCANTimeoutMs);

        turret.selectProfileSlot(0, 0);
        turret.config_kP(0, Constants.Turret.TURRET_KP, Constants.kLongCANTimeoutMs);
        turret.config_kI(0, Constants.Turret.TURRET_KI, Constants.kLongCANTimeoutMs);
        turret.config_kD(0, Constants.Turret.TURRET_KD, Constants.kLongCANTimeoutMs);
        turret.config_kF(0, Constants.Turret.TURRET_KF, Constants.kLongCANTimeoutMs);

        turret.configMotionCruiseVelocity((int)(Constants.Turret.TURRET_MAXSPEED), Constants.kLongCANTimeoutMs);
        turret.configMotionAcceleration((int)(Constants.Turret.TURRET_MAXSPEED * 6.0), Constants.kLongCANTimeoutMs);
        turret.configMotionSCurveStrength(0);

        turret.configForwardSoftLimitThreshold(turretDegreesToInternalEncUnits(Constants.Turret.TURRET_MAXCONTROLANGLE), Constants.kLongCANTimeoutMs);
        turret.configReverseSoftLimitThreshold(turretDegreesToInternalEncUnits(Constants.Turret.TURRET_MINCONTROLANGLE), Constants.kLongCANTimeoutMs);
        turret.configForwardSoftLimitEnable(true, Constants.kLongCANTimeoutMs);
        turret.configReverseSoftLimitEnable(true, Constants.kLongCANTimeoutMs);

        System.out.println("Turret max soft limit: " + turretDegreesToInternalEncUnits(Constants.Turret.TURRET_MAXCONTROLANGLE));
        System.out.println("Turret min soft limit: " + turretDegreesToInternalEncUnits(Constants.Turret.TURRET_MINCONTROLANGLE));

        turret.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 50, Constants.kLongCANTimeoutMs); //was 20 and no timeout
        turret.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, Constants.kLongCANTimeoutMs); //was 50 and no timeout

        turret.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 1000, Constants.kLongCANTimeoutMs);
        turret.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 1000, Constants.kLongCANTimeoutMs);
        turret.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 1000, Constants.kLongCANTimeoutMs);
        turret.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 1000, Constants.kLongCANTimeoutMs);
        turret.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 1000, Constants.kLongCANTimeoutMs);
        turret.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 1000, Constants.kLongCANTimeoutMs);
        turret.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 1000, Constants.kLongCANTimeoutMs);

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
        targetAngle = boundToTurretRange(angle);
        /*if (ActuatingHood.getInstance().isStowed())
            targetAngle = closestPole();
            */
        periodicIO.controlMode = ControlMode.MotionMagic;
        periodicIO.demand = turretDegreesToInternalEncUnits(targetAngle);
        // System.out.println("Demand is: " + periodicIO.demand);
    }

    // private void setVisionAngle(double angle) {
    //     targetAngle = boundToTurretRange(angle);
    //     /*if (ActuatingHood.getInstance().isStowed())
    //         targetAngle = closestPole();
    //         */
    //     periodicIO.controlMode = ControlMode.Position;
    //     periodicIO.demand = turretDegreesToInternalEncUnits(targetAngle);
    //     System.out.println("Demand is: " + periodicIO.demand);
    // }
    
    private boolean inRange(double value, double min, double max) {
        return min <= value && value <= max;
    }

    public boolean inVisionRange(double angle) {
        for (double[] range : Constants.Turret.kVisionRanges) {
            if (inRange(angle, range[0], range[1]))
                return true;
        }
        return false;
    }

    public boolean inVisionRange() {
        return inVisionRange(getAngle());
    }

    public boolean inTurretRange(double angle) {
        angle = Util.boundAngle0to360Degrees(angle);
        if (!inRange(angle, Constants.Turret.TURRET_MINCONTROLANGLE, Constants.Turret.TURRET_MAXCONTROLANGLE)) {
            angle = (angle < Constants.Turret.TURRET_MINCONTROLANGLE) ? (angle + 360.0) : (angle - 360.0);
            return inRange(angle, Constants.Turret.TURRET_MINCONTROLANGLE, Constants.Turret.TURRET_MAXCONTROLANGLE);
        }
        return true;
    }

    /** 
     * Takes an angle from any scope and finds the equivalent angle in the turret's range of motion or,
     * if one doesn't exist, returns the turret soft limit closest to the desired angle.
    */
    public double boundToTurretRange(double angle) {
        angle = Util.placeInAppropriate0To360Scope(getAngle(), angle);

        if (!inRange(angle, Constants.Turret.TURRET_MINCONTROLANGLE, Constants.Turret.TURRET_MAXCONTROLANGLE)) {
            angle = (angle < Constants.Turret.TURRET_MINCONTROLANGLE) ? (angle + 360.0) : (angle - 360.0);
            if (inRange(angle, Constants.Turret.TURRET_MINCONTROLANGLE, Constants.Turret.TURRET_MAXCONTROLANGLE))
                return angle;
            return (Math.abs(Rotation2d.fromDegrees(angle).distance(Rotation2d.fromDegrees(Constants.Turret.TURRET_MINCONTROLANGLE)))
                < Math.abs(Rotation2d.fromDegrees(angle).distance(Rotation2d.fromDegrees(Constants.Turret.TURRET_MAXCONTROLANGLE))))
                ? Constants.Turret.TURRET_MINCONTROLANGLE : Constants.Turret.TURRET_MAXCONTROLANGLE;
        }

        return angle;
    }

    public void setPosition(double angle) {
        setState(State.POSITION);
        setAngle(angle);
    }

    public void gyroComp() {
        if (currentState != State.GYRO_COMP) 
            setState(State.GYRO_COMP);
        gyroLockedHeading = robotState.getLatestFieldToVehicle().getValue().getRotation().getDegrees();
        gyroLockedTurretHeading = getAngle();
    }

    public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(getAngle());
    }

    public void lockAngle() {
        setState(State.AWAITING_LOCK);
    }

    public void startVision(boolean aimInnerPort) {
        this.aimInnerPort = aimInnerPort;
        setState(State.VISION);
        Optional<ShooterAimingParameters> aim = robotState.getAimingParameters(aimInnerPort);
        if (aim.isPresent()) {
            System.out.println("Vision Target Range: " + aim.get().getRange() + " Vision Target Angle: " + aim.get().getTurretAngle().getDegrees());
            setAngle(aim.get().getTurretAngle().getDegrees());
        } else {
            System.out.println("No visible or cached target");
        }
    }

    public void startExperimentalVision() {
        // this.aimInnerPort = aimInnerPort;
        setState(State.EXPERIMENTAL_VISION);
        Optional<ShooterAimingParameters> aim_exp = robotState.getExperimentalAimingParameters();
        if (aim_exp.isPresent()) {
            System.out.println("Vision Target Range: " + aim_exp.get().getRange() + " Vision Target Angle: " + aim_exp.get().getTurretAngle().getDegrees());
            setAngle(aim_exp.get().getTurretAngle().getDegrees());
        } else {
            System.out.println("No visible or cached target");
        }
    }

    public void setOpenLoop(double output) {
        periodicIO.controlMode = ControlMode.PercentOutput;
        periodicIO.demand = /*Hood.getInstance().isStowed() ? 0.0 : */Util.scaledDeadband(output, 1.0, 0.25) * 0.5;
        setState(State.OPEN_LOOP);
    }
    
    public boolean isOpenLoop() {
        return currentState == State.OPEN_LOOP;
    }

    public boolean isTracking() {
        return (currentState == State.VISION || currentState == State.EXPERIMENTAL_VISION);
    }
    
    public boolean hasReachedAngle() {
        return Math.abs(getAngle() - targetAngle) < Constants.Turret.kAngleTolerance;
    }

    public double getAngle() {
        return internalEncUnitsToTurretDegrees(periodicIO.position);
    }

    @Override
    public synchronized void readPeriodicInputs() {
        periodicIO.position = (int)turret.getSelectedSensorPosition(0);
        if (Settings.debugTurret()) {
            periodicIO.velocity = (int)turret.getSelectedSensorVelocity(0);
            periodicIO.voltage = turret.getMotorOutputVoltage();
            periodicIO.current = turret.getOutputCurrent();
        }
    }
    
    @Override
    public void writePeriodicOutputs() {
        if (zeroedAbsolutely) {
            turret.set(periodicIO.controlMode, periodicIO.demand);
        }
    }

    private final Loop loop = new Loop() {

        @Override
        public void onStart(double timestamp) {
            turret.setNeutralMode(NeutralMode.Brake);
        }

        @Override
        public void onLoop(double timestamp) {
            // System.out.println("Starting ONLOOP METHOD");
            switch(currentState) {
                case VISION:
                    // System.out.println("Current state is VISION");
                    Optional<ShooterAimingParameters> aim  = robotState.getAimingParameters(aimInnerPort);
                    if (aim.isPresent()) {
                        setAngle(aim.get().getTurretAngle().getDegrees());
                    }
                    break;
                case EXPERIMENTAL_VISION:
                    // System.out.println("Current state is VISION");
                    Optional<ShooterAimingParameters> aim_exp  = robotState.getExperimentalAimingParameters();
                    if (aim_exp.isPresent()) {
                        setAngle(aim_exp.get().getTurretAngle().getDegrees());
                    }
                    break;
                case GYRO_COMP:
                    // System.out.println("Current state is GYROCOMP");
                    setAngle(gyroLockedTurretHeading - (robotState.getLatestFieldToVehicle().getValue().getRotation().getDegrees() - gyroLockedHeading));
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
            turret.setNeutralMode(NeutralMode.Coast);
        }
    };

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(loop);
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Turret Angle", getAngle());
        //SmartDashboard.putNumber("Turret Current", periodicIO.current);
        //SmartDashboard.putNumber("Turret Voltage", periodicIO.voltage);
        //SmartDashboard.putBoolean("Turret Zeroed Absolutely", zeroedAbsolutely);
        // SmartDashboard.putNumber("Turret Absolute Angle", getAbsoluteEncoderDegrees());
        if (Settings.debugTurret()) {
            SmartDashboard.putString("Turret State", currentState.toString());
            // SmartDashboard.putNumber("Turret Absolute Frequency", encoder.getFrequency());
            SmartDashboard.putNumber("Turret Encoder", periodicIO.position);
            SmartDashboard.putNumber("Turret Velocity", periodicIO.velocity);
            SmartDashboard.putNumber("Turret Angle Error", internalEncUnitsToTurretDegrees(turret.getClosedLoopError(0)));
            if (turret.getControlMode() == ControlMode.MotionMagic) {
                SmartDashboard.putNumber("Turret Setpoint", turret.getClosedLoopTarget(0));
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
                turret.configMotionCruiseVelocity((int)(Constants.Turret.TURRET_MAXSPEED * speedScalar), Constants.kCANTimeoutMs);
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

    public Request startVisionRequest(boolean aimInnerPort) {
        System.out.println("Start Vision Request");
        return new Request(){
            
            @Override
            public void act() {
                System.out.println("ACTING");
                startVision(aimInnerPort);
            }
            
            @Override
            public boolean isFinished() {
                if (hasReachedAngle()) {
                    DriverStation.reportError("Turret Request Finished", false);
                    return true;
                }
                return false;
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
                if (hasReachedAngle()) {
                    DriverStation.reportError("Turret Request Finished", false);
                    return true;
                }
                return false;
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

    public int turretDegreesToInternalEncUnits(double turretAngle) {
        return (int) ((turretAngle / 360.0) * Constants.Turret.kInternalEncToOutputRatio * 2048.0 );
    }

    public double internalEncUnitsToTurretDegrees(double encUnits) {
        return (encUnits / 2048.0) / Constants.Turret.kInternalEncToOutputRatio * 360.0;
    }

    public synchronized void resetToAbsolute() {
        if (!zeroedAbsolutely) {
            //System.out.println("Turret Encoder Connected: " + isEncoderConnected());
            if (RobotBase.isReal()) {
                //DriverStation.reportError("TURRET WAS RESET TO ABSOLUTE WITH THE MAG ENCODER", false);
                double absolutePosition = Util.boundAngle0to360Degrees(Constants.Turret.kEncoderStartingAngle);
                if (absolutePosition > Constants.Turret.TURRET_MAXINITIALANGLE)
                    absolutePosition -= 360.0;
                else if (absolutePosition < Constants.Turret.TURRET_MININITIALLANGLE)
                    absolutePosition += 360.0;
                if (!inRange(absolutePosition, Constants.Turret.TURRET_MININITIALLANGLE, Constants.Turret.TURRET_MAXINITIALANGLE)) {
                    DriverStation.reportError("Turret angle is out of bounds", false);
                    hasEmergency = true;
                }
                turret.setSelectedSensorPosition(turretDegreesToInternalEncUnits(absolutePosition));
                // turret.setSelectedSensorPosition(turretDegreesToInternalEncUnits(absolutePosition), 0, Constants.kCANTimeoutMs);
                //System.out.println("Turret Absolute angle: " + getAbsoluteEncoderDegrees() + ", encoder offset: " + Constants.Turret.kEncoderStartingAngle + ", difference: " + (getAbsoluteEncoderDegrees() - Constants.Turret.kEncoderStartingAngle) + ", degreesToEncUnits: " + turretDegreesToInternalEncUnits(getAbsoluteEncoderDegrees() - Constants.Turret.kEncoderStartingAngle));
            } else {
                DriverStation.reportError("Turret encoder NOT DETECTED: CURRENT POSITION SET TO 0", false);
                turret.setSelectedSensorPosition(turretDegreesToInternalEncUnits(180.0));
                // turret.setSelectedSensorPosition(0, 0, Constants.kCANTimeoutMs);
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

    public synchronized void zeroedTurret() {
        System.out.println("Turret Zeroed");
        System.out.println("Turret Absolute angle: 0.0" + ", encoder offset: " + Constants.Turret.kEncoderStartingAngle + ", difference: 0.0");
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
