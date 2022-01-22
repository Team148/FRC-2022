/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystems;

import java.util.Optional;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.FeedbackDevice;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
// import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import frc.Constants;
import frc.RobotMap;
import frc.loops.Loop;
import frc.loops.ILooper;
import frc.RobotState;
import frc.subsystems.requests.Request;
import frc.vision.ShooterAimingParameters;
import com.team1323.lib.util.Util;
// import com.team254.drivers.LazyTalonFX;
import com.team1323.lib.util.SynchronousPIDF;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.PWM;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.DriverStation;  
// import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * An adjustable shooter hood, powered by a BAG motor
 */
public class MotorizedHood extends Subsystem {
    private static MotorizedHood instance = null;
    public static MotorizedHood getInstance() {
        if (instance == null)
            instance = new MotorizedHood();
        return instance;
    }

    // LazyTalonFX hood;
    // DutyCycle encoder;
    PWM left_servo;
    PWM right_servo;
    CANCoder encoder;
    RobotState robotState;

    SynchronousPIDF pidf;

    double lastUpdateTimestamp = 0;
    
    private boolean isEncoderFlipped = false;
    // private boolean zeroedAbsolutely = false;

    private double desiredAngle = 0;

    enum ControlMode { OPEN_LOOP, POSITION }


    PeriodicIO periodicIO = new PeriodicIO();



    public MotorizedHood() {
        // hood = new LazyTalonFX(RobotMap.HOOD_TALON);
        // encoder = new DutyCycle(new DigitalInput(RobotMap.HOOD_ENCODER));
        robotState = RobotState.getInstance();

        left_servo = new PWM(RobotMap.SERVO_1);
        right_servo = new PWM(RobotMap.SERVO_2);

        left_servo.setBounds(1.7, 1.52, 1.5, 1.48, 1.3);   //bounds for SAVOX
        right_servo.setBounds(1.7, 1.52, 1.5, 1.48, 1.3);  //bounds for SAVOX

        encoder = new CANCoder(RobotMap.HOOD_CAN_PORT);
        encoder.configFactoryDefault();
        encoder.configMagnetOffset(Constants.MotorizedHood.kEncoderOffset);

        pidf = new SynchronousPIDF(Constants.MotorizedHood.kP, Constants.MotorizedHood.kI, Constants.MotorizedHood.kD);
        pidf.setDeadband(Constants.MotorizedHood.kAngleDeadband);
        pidf.setInputRange(Constants.MotorizedHood.kMinInitialAngle, Constants.MotorizedHood.kMaxControlAngle);



        pidf.setSetpoint(0.0);

        // hood.configVoltageCompSaturation(12.0, Constants.kCANTimeoutMs);
        // hood.enableVoltageCompensation(true);
        // hood.setInverted(false);
        // setEncoderPhase(true);

        // hood.setNeutralMode(NeutralMode.Brake);

        // hood.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        // setCurrentLimit(10.0);

        // hood.selectProfileSlot(0, 0);
        // hood.config_kP(0, Constants.MotorizedHood.kP, Constants.kCANTimeoutMs);
        // hood.config_kI(0, Constants.MotorizedHood.kI, Constants.kCANTimeoutMs);
        // hood.config_kD(0, Constants.MotorizedHood.kD, Constants.kCANTimeoutMs);
        // hood.config_kF(0, Constants.MotorizedHood.kF, Constants.kCANTimeoutMs);

        // hood.configMotionCruiseVelocity((int)(Constants.MotorizedHood.kMaxSpeed * 1.0), Constants.kCANTimeoutMs);
        // hood.configMotionAcceleration((int)(Constants.MotorizedHood.kMaxSpeed * 3.0), Constants.kCANTimeoutMs);

        // hood.configForwardSoftLimitThreshold(hoodAngleToEncUnits(Constants.MotorizedHood.kMaxControlAngle), Constants.kCANTimeoutMs);
        // hood.configReverseSoftLimitThreshold(hoodAngleToEncUnits(Constants.MotorizedHood.kMinControlAngle), Constants.kCANTimeoutMs);
        // hood.configForwardSoftLimitEnable(true, Constants.kCANTimeoutMs);
        // hood.configReverseSoftLimitEnable(true, Constants.kCANTimeoutMs);

        // hood.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
        // hood.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 50);

        // resetToAbsolute();
        //hood.setSelectedSensorPosition(0);

        // setOpenLoop(0.0);
    }

    // private void setCurrentLimit(double amps) {
    //     SupplyCurrentLimitConfiguration currentLimitConfiguration = new SupplyCurrentLimitConfiguration(true, amps, amps, 10);
    //     hood.configSupplyCurrentLimit(currentLimitConfiguration, Constants.kCANTimeoutMs);
    // }

    // private void setEncoderPhase(boolean phase) {
    //     isEncoderFlipped = phase;
    // }

    private double getAbsoluteEncoderDegrees() {
        // return (isEncoderFlipped ? -1.0 : 1.0) * encoder.getOutput() * 360.0;
        return (isEncoderFlipped ? -1.0 : 1.0) * encoder.getAbsolutePosition();

    }

    private boolean isEncoderConnected() {
        if (RobotBase.isReal()) {
            //System.out.println("Robot Base: " + RobotBase.isReal() + " Encoder Freq: " + encoder.getFrequency());
            // return (encoder.getFrequency() != 0) ? true : false;
            return (encoder.getBusVoltage() != 0) ? true : false;

        }
        return true;
    }

    // public double encUnitsToDegrees(double encUnits) {
    //     return encUnits / Constants.MotorizedHood.kTicksPerDegree;
    // }

    // public int degreesToEncUnits(double degrees) {
    //     return (int)(degrees * Constants.MotorizedHood.kTicksPerDegree);
    // }

    // public double encUnitsToHoodAngle(double encUnits) {
    //     return encUnitsToDegrees(encUnits);
    // }

    // public int hoodAngleToEncUnits(double degrees) {
    //     return (int)(degreesToEncUnits(degrees));
    // }

    public double getRawAngle(){
        return periodicIO.position;
    }

    public double getAngle() {
        // return encUnitsToHoodAngle(periodicIO.position);
        return getRawAngle() / Constants.MotorizedHood.kEncoderRatio;
    }

    public void setAngle(double angle) {
        periodicIO.controlMode = ControlMode.POSITION;
        desiredAngle = Util.limit(angle, Constants.MotorizedHood.kMinControlAngle, Constants.MotorizedHood.kMaxControlAngle);
        pidf.reset();

        pidf.setSetpoint(desiredAngle) ;

        // periodicIO.controlMode = ControlMode.MotionMagic;
        // periodicIO.demand = hoodAngleToEncUnits(angle);
    }

    // public void lockAngle() {
    //     periodicIO.controlMode = ControlMode.MotionMagic;
    //     periodicIO.demand = periodicIO.position;
    // }

    public boolean hasReachedAngle() {
        // return periodicIO.controlMode == ControlMode.MotionMagic && 
            // Math.abs(encUnitsToDegrees(periodicIO.demand - periodicIO.position)) <= Constants.MotorizedHood.kAngleTolerance;
            return Math.abs(desiredAngle - periodicIO.position) <= Constants.MotorizedHood.kAngleTolerance;
    }

    public void setOpenLoop(double percentOutput) {
        periodicIO.controlMode = ControlMode.OPEN_LOOP;
        periodicIO.demand = percentOutput;
    }

    public void visionExtension() {
        double farDistanceMovement = 170.0;
        double closeDistanceMovement = 100.0;
        Optional<ShooterAimingParameters> aim = robotState.getAimingParameters(false);
        if (aim.isPresent() && robotState.seesTarget() && aim.get().getRange() >= farDistanceMovement) {
            //System.out.println("HOOD STATE SET: FAR" + " VISION DISTANCE: " + aim.get().getRange());
            setAngle(Constants.MotorizedHood.CompFarAngle);
        } else if (aim.isPresent() && robotState.seesTarget() && aim.get().getRange() < farDistanceMovement && aim.get().getRange() >= closeDistanceMovement) {
            //System.out.println("HOOD STATE SET: MID" + " VISION DISTANCE: " + aim.get().getRange());
            setAngle(Constants.MotorizedHood.CompMidAngle);
        } else if (aim.isPresent() && robotState.seesTarget() && aim.get().getRange() < closeDistanceMovement) {
            //System.out.println("HOOD STATE SET: MID" + " VISION DISTANCE: " + aim.get().getRange());
            setAngle(Constants.MotorizedHood.CompCloseAngle);
        }
    }

    public Request angleRequest(double angle) {
        return new Request(){
        
            @Override
            public void act() {
                setAngle(angle);
            }

            @Override
            public boolean isFinished() {
                return hasReachedAngle();
            }
            
        };
    }

    public Request openLoopRequest(double percentOutput) {
        return new Request(){
        
            @Override
            public void act() {
                setOpenLoop(percentOutput);
            }

        };
    }

    @Override
    public void readPeriodicInputs() {
        // periodicIO.position = hood.getSelectedSensorPosition(0);
        periodicIO.position = encoder.getAbsolutePosition();
    }

    @Override
    public void writePeriodicOutputs() {
        // hood.set(periodicIO.controlMode, periodicIO.demand);
        left_servo.setSpeed(-periodicIO.demand);
        right_servo.setSpeed(periodicIO.demand);
        // left_servo.setSpeed(-1);
        // right_servo.setSpeed(1);

    }

    // public void resetToAbsolute() {
    //     if (!zeroedAbsolutely) {
    //         if (isEncoderConnected() && RobotBase.isReal()) {
    //             //DriverStation.reportError("HOOD WAS RESET TO ABSOLUTE WITH THE MAG ENCODER", false);
    //             double absolutePosition = Util.boundAngle0to360Degrees(Constants.MotorizedHood.kHoodStartingAngle + (getAbsoluteEncoderDegrees() - Constants.MotorizedHood.kEncStartingAngle));
    //             if (absolutePosition > Constants.MotorizedHood.kMaxInitialAngle)
    //                 absolutePosition -= 360.0;
    //             else if (absolutePosition < Constants.MotorizedHood.kMinInitialAngle)
    //                 absolutePosition += 360.0;
    //             if(absolutePosition > Constants.MotorizedHood.kMaxInitialAngle || absolutePosition < Constants.MotorizedHood.kMinInitialAngle){
    //                 DriverStation.reportError("Hood angle is out of bounds", false);
    //                 hasEmergency = true;
    //             }
    //             SmartDashboard.putNumber("Hood Zero", degreesToEncUnits(absolutePosition));
    //             hood.setSelectedSensorPosition(degreesToEncUnits(absolutePosition), 0, Constants.kCANTimeoutMs);
    //             //System.out.println("Hood Absolute angle: " + getAbsoluteEncoderDegrees() + ", encoder offset: " + Constants.MotorizedHood.kEncStartingAngle + ", difference: " + (getAbsoluteEncoderDegrees() - Constants.MotorizedHood.kEncStartingAngle) + ");
    //         } else {
    //             DriverStation.reportError("Hood encoder NOT DETECTED: CURRENT POSITION SET TO 0", false);
    //             hood.setSelectedSensorPosition(degreesToEncUnits(Constants.MotorizedHood.kHoodStartingAngle), 0, Constants.kCANTimeoutMs);
    //         }
    //     }
    // }

    // public synchronized void zeroHood() {
    //     System.out.println("Hood Zeroed");
    //     System.out.println("Hood Absolute angle: " + getAbsoluteEncoderDegrees() + ", encoder offset: " + Constants.MotorizedHood.kEncStartingAngle + ", difference: " + (getAbsoluteEncoderDegrees() - Constants.MotorizedHood.kEncStartingAngle) + ", degreesToEncUnits: " + degreesToEncUnits(getAbsoluteEncoderDegrees() - Constants.MotorizedHood.kEncStartingAngle));
    //     zeroedAbsolutely = true;
    // }

    private final Loop loop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            periodicIO.controlMode = ControlMode.OPEN_LOOP;
            periodicIO.demand = 0.0;


        }

        @Override
        public void onLoop(double timestamp) {

            if(periodicIO.controlMode == ControlMode.POSITION){
                double dt = lastUpdateTimestamp - timestamp;

                periodicIO.demand = pidf.calculate(getAngle(), dt);

            }
            else{

            }

            lastUpdateTimestamp = timestamp;


        }

        @Override
        public void onStop(double timestamp) {

            periodicIO.controlMode = ControlMode.OPEN_LOOP;
            periodicIO.demand = 0.0;

        }



    };

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(loop);
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Hood Absolute Encoder", getAbsoluteEncoderDegrees()); // -281 -234
        // SmartDashboard.putNumber("Hood Encoder", periodicIO.position); // -8000
        SmartDashboard.putNumber("Hood Angle", getAngle()); // -17 -66
        SmartDashboard.putNumber("Hood Servo Power", periodicIO.demand);
        SmartDashboard.putNumber("Hood Desired Angle", desiredAngle);

        //SmartDashboard.putNumber("Hood Error", encUnitsToDegrees(periodicIO.demand - periodicIO.position));
        //SmartDashboard.putNumber("Hood Velocity", hood.getSelectedSensorVelocity(0));
    }

    @Override
    public void stop() {
        setOpenLoop(0.0);
    }

    public class PeriodicIO {
        public double position = 0.0;

        public double demand = 0.0;
        public ControlMode controlMode = ControlMode.POSITION;
    }


}
