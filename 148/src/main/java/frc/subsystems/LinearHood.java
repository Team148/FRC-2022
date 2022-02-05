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

import com.team1323.lib.util.LinearServo;

/**
 * An adjustable shooter hood, powered by a BAG motor
 */
public class LinearHood extends Subsystem {
    private static LinearHood instance = null;
    public static LinearHood getInstance() {
        if (instance == null)
            instance = new LinearHood();
        return instance;
    }

    // LazyTalonFX hood;
    // DutyCycle encoder;
    LinearServo left_servo;
    LinearServo right_servo;
    
    RobotState robotState;

    SynchronousPIDF pidf;

    double lastUpdateTimestamp = 0;

    private double desiredAngle = 0;

    enum ControlMode { OPEN_LOOP, POSITION }


    PeriodicIO periodicIO = new PeriodicIO();



    public LinearHood() {
        // hood = new LazyTalonFX(RobotMap.HOOD_TALON);
        // encoder = new DutyCycle(new DigitalInput(RobotMap.HOOD_ENCODER));
        robotState = RobotState.getInstance();

        left_servo = new LinearServo(RobotMap.SERVO_1, 170, 24);
        right_servo = new LinearServo(RobotMap.SERVO_2, 170, 24);

    }

    public void setAngle(double angle) {
        periodicIO.controlMode = ControlMode.POSITION;
        desiredAngle = Util.limit(angle, Constants.MotorizedHood.kMinControlAngle, Constants.MotorizedHood.kMaxControlAngle);
        pidf.reset();

        pidf.setSetpoint(desiredAngle) ;

        // periodicIO.controlMode = ControlMode.MotionMagic;
        // periodicIO.demand = hoodAngleToEncUnits(angle);
    }

    public void setOpenLoop(double percentOutput) {
        periodicIO.controlMode = ControlMode.OPEN_LOOP;
        periodicIO.demand = percentOutput;
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
        periodicIO.position = left_servo.getPosition();
    }

    @Override
    public void writePeriodicOutputs() {
        // hood.set(periodicIO.controlMode, periodicIO.demand);
        left_servo.setSpeed(-periodicIO.demand);
        right_servo.setSpeed(periodicIO.demand);
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