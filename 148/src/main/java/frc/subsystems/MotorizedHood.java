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
import com.team1323.lib.util.SynchronousPIDF;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.team1323.lib.util.LinearServo;

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

    LinearServo leftServo;
    LinearServo rightServo;

    RobotState robotState;

    double lastUpdateTimestamp = 0;

    private double desiredAngle = 0;

    enum ControlMode { OPEN_LOOP, POSITION }


    PeriodicIO periodicIO = new PeriodicIO();



    public MotorizedHood() {
        // hood = new LazyTalonFX(RobotMap.HOOD_TALON);
        // encoder = new DutyCycle(new DigitalInput(RobotMap.HOOD_ENCODER));
        robotState = RobotState.getInstance();

        leftServo = new LinearServo(RobotMap.SERVO_1, 170, 24);
        rightServo = new LinearServo(RobotMap.SERVO_2, 170, 24);

    }

    public void setServoPosition(double setpoint) {
        leftServo.linearsetPosition(setpoint);
        rightServo.linearsetPosition(setpoint);
    }

    public double getRawAngle(){
        return 0.0;
        // return periodicIO.position;
    }

    public void setAngle(double angle) {
        
    }

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
        double farDistanceMovement = 165.0;
        double closeDistanceMovement = 30.0;
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
    public void writePeriodicOutputs() {
        // hood.set(periodicIO.controlMode, periodicIO.demand);
        leftServo.setSpeed(-periodicIO.demand);
        rightServo.setSpeed(periodicIO.demand);
        // left_servo.setSpeed(-1);
        // right_servo.setSpeed(1);

    }

    private final Loop loop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            periodicIO.controlMode = ControlMode.OPEN_LOOP;
            periodicIO.demand = 0.0;

        }

        @Override
        public void onLoop(double timestamp) {

            leftServo.updateCurPos();
            rightServo.updateCurPos();
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
        // SmartDashboard.putNumber("Hood Absolute Encoder", getAbsoluteEncoderDegrees()); // -281 -234
        // SmartDashboard.putNumber("Hood Encoder", periodicIO.position); // -8000
        // SmartDashboard.putNumber("Hood Angle", getAngle()); // -17 -66
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

