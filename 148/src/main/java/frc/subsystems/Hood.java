package frc.subsystems;

import frc.Constants;
import frc.Robot;
import frc.RobotMap;
import frc.loops.Loop;
import frc.loops.ILooper;

import com.team254.lib.geometry.Rotation2d;
import com.team1323.lib.util.SynchronousPIDF;

import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The Hood assembly controls the launch angle of the ball. Additionally, it
 * must be lowered whenever the robot is crossing a defense. There are several
 * parameters accessible to the rest of the robot code: the hood angle, whether
 * or not the hood is stowed.
 * 
 * The ball is first picked up with the Intake then is fed to the Flywheel with
 * the HoodRoller. The Turret controls the direction that the ball is fired at.
 * Finally, the Hood controls the output angle and, conversely, trajectory.
 */
public class Hood extends Subsystem {
    private static Hood instance = null;

    public static Hood getInstance(){
		if(instance == null){
			instance = new Hood();
		}
		return instance;
	}
    private final PWM left_servo_;
    private final PWM right_servo_;
    private final CANCoder encoder_;
    private boolean has_homed_;
    private final SynchronousPIDF pidf_;

    double lastUpdateTimestamp = 0;


    enum ControlMode {
        HOMING, OPEN_LOOP, POSITION
    }

    ControlMode control_mode_;

    private final Loop loop = new Loop() {

        static final double kHomingTimeSeconds = 0.1;
        ControlMode last_iteration_control_mode_ = ControlMode.OPEN_LOOP;
        double homing_start_time_ = 0;

        @Override
        public void onStart(double timestamp) {
            synchronized (Hood.this) {
                if (!has_homed_) {
                    control_mode_ = ControlMode.HOMING;
                }
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Hood.this) {

                double dt = lastUpdateTimestamp - timestamp;
                if (control_mode_ == ControlMode.HOMING) {
                    if (control_mode_ != last_iteration_control_mode_) {
                        startHoming();
                        homing_start_time_ = Timer.getFPGATimestamp();
                    } else if (Timer.getFPGATimestamp() >= homing_start_time_ + kHomingTimeSeconds) {
                        stopHoming(true);
                        
                    }
                } else if (control_mode_ == ControlMode.POSITION) {
                    pidf_.setSetpoint(mCurrentDesiredSetpoint);
                    // System.out.println("ControlMode.POSITION, angle: "+Robot.getTurret().getUnboundedAngle());
                    if(Math.abs(Robot.getTurret().getAngle()-Constants.Hood.SAFE_TURRET_ANGLE_FORWARD) < Constants.Hood.SAFE_TURRET_TOLERANCE){
                        // turret angle is safe to move hood ANYWHERE
                        set(pidf_.calculate(getAngle().getUnboundedDegrees(), dt));
                    }
                    else if(Math.abs(Robot.getTurret().getAngle()-Constants.Hood.SAFE_TURRET_ANGLE_BACKWARD_POSITIVE) < Constants.Hood.SAFE_TURRET_TOLERANCE) {
                        // turret angle is safe to move hood ANYWHERE
                        set(pidf_.calculate(getAngle().getUnboundedDegrees(), dt));
                    }
                    else if(Math.abs(Robot.getTurret().getAngle()-Constants.Hood.SAFE_TURRET_ANGLE_BACKWARD_NEGATIVE) < Constants.Hood.SAFE_TURRET_TOLERANCE) {
                        // turret angle is safe to move hood ANYWHERE
                        set(pidf_.calculate(getAngle().getUnboundedDegrees(), dt));
                    }
                    else{
                        if(getSetpoint() > Constants.Hood.SAFE_HOOD_ANGLE){
                            // ALWAYS safe to move hood up
                            set(pidf_.calculate(getAngle().getUnboundedDegrees(), dt));
                        }else{
                            // set setpoint to a safe degree
                            pidf_.setSetpoint(Constants.Hood.SAFE_HOOD_SETPOINT);
                            set(pidf_.calculate(getAngle().getUnboundedDegrees(), dt));
                        }
                    }
                }
                last_iteration_control_mode_ = control_mode_;
                lastUpdateTimestamp = timestamp;

                // if(!encoder_.getFaults(CANCoderFaults.))
                //     System.out.println("Lost Hood Encoder");
                // System.out.println("OnLoop Hood");
                // System.out.println(getAngle().getUnboundedDegrees());
            }


        }



        @Override
        public void onStop(double timestamp) {
            synchronized (Hood.this) {
                if (control_mode_ == ControlMode.HOMING) {
                    stopHoming(false);
                }
            }
        }
    };

    private Hood() {
        left_servo_ = new PWM(RobotMap.SERVO_1);
        right_servo_ = new PWM(RobotMap.SERVO_2);

        left_servo_.setBounds(1.7, 1.52, 1.5, 1.48, 1.3);   //bounds for SAVOX
        right_servo_.setBounds(1.7, 1.52, 1.5, 1.48, 1.3);  //bounds for SAVOX

        // test_servo_ = new ContinuousRotationServo(Constants.kTestServoPWM);
        // test_servo_.set(0.0);
        encoder_ = new CANCoder(RobotMap.HOOD_CAN_PORT);

        encoder_.configFactoryDefault();
        encoder_.configMagnetOffset(Constants.Hood.HOOD_ENCODER_OFFSET);
        // encoder_.setDistancePerRotation(360.0);
        pidf_ = new SynchronousPIDF(Constants.Hood.HOOD_KP, Constants.Hood.HOOD_KI, Constants.Hood.HOOD_KP);
        pidf_.setDeadband(Constants.Hood.HOOD_DEADBAND);
        pidf_.setInputRange(Constants.Hood.MIN_HOOD_ANGLE, Constants.Hood.MAX_HOOD_ANGLE);

        has_homed_ = false;
        pidf_.setSetpoint(Constants.Hood.MIN_HOOD_ANGLE);
        control_mode_ = ControlMode.OPEN_LOOP;
    }

    // Loop getLoop() {
    //     return loop;
    // }

    /**
     * Sets the angle of the hood to a specified angle.
     * 
     * @param A set angle
     */
    // synchronized void setDesiredAngle(final Rotation2d angle) {
    public void setDesiredAngle(Rotation2d angle) {
        if (control_mode_ != ControlMode.HOMING && control_mode_ != ControlMode.POSITION) {
            control_mode_ = ControlMode.POSITION;
            pidf_.reset();
            
        }
        double desired_angle = angle.getUnboundedDegrees();
        if(desired_angle < Constants.Hood.MIN_HOOD_ANGLE) desired_angle = Constants.Hood.MIN_HOOD_ANGLE;

        pidf_.setSetpoint(desired_angle);
        mCurrentDesiredSetpoint = desired_angle;
        // System.out.println("Desired Angle: " + desired_angle);
    }

    private double mCurrentDesiredSetpoint;

    /**
     * Gets the current angle of the hood.
     * 
     * @return The hood's current angle.
     */
    public synchronized Rotation2d getAngle() {
        // return Rotation2d.fromDegrees(encoder_.get() * 360.0 * Constants.kHoodGearReduction + Constants.kMinHoodAngle);
        return Rotation2d.fromDegrees((encoder_.getAbsolutePosition()+ Constants.Hood.HOOD_ANGLE_OFFSET) *  Constants.Hood.HOOD_GEAR_REDUCTION + Constants.Hood.MIN_HOOD_ANGLE );

    }

    private synchronized void set(double power) {
        left_servo_.setSpeed(-power);
        right_servo_.setSpeed(power);

        // System.out.println(power);
    }

    synchronized void setOpenLoop(double power) {
        if (control_mode_ != ControlMode.HOMING) {
            set(power);
            control_mode_ = ControlMode.OPEN_LOOP;
        }
    }

    /**
     * Sets the hood state such that it begins retracting
     */
    public synchronized void homeSystem() {
        control_mode_ = ControlMode.HOMING;
    }

    /**
     * Makes the hood assembly begin to retract, or home.
     */
    synchronized void startHoming() {
        control_mode_ = ControlMode.HOMING;
        set(-1.0);
    }

    /**
     * Changes the control state of the Hood assembly if the hood is fully
     * retracted. If not, the Hood state is set to Open Loop.
     * 
     * @param If the hood has fully retracted.
     */
    synchronized void stopHoming(boolean success) {
        if (success) {
            has_homed_ = true;
            control_mode_ = ControlMode.POSITION;
            zeroSensors();
        } else {
            control_mode_ = ControlMode.OPEN_LOOP;
        }
        set(0);
    }

    public synchronized boolean hasHomed() {
        return has_homed_;
    }

    public synchronized double getSetpoint() {
        return pidf_.getSetpoint();
    }

    /**
     * @return If the hood position is within a set tolerance to a specified value.
     */
    public synchronized boolean isOnTarget() {
        return (has_homed_ && control_mode_ == ControlMode.POSITION
                && Math.abs(pidf_.getError()) < Constants.Hood.HOOD_ON_TARGET_TO_TOLERANCE);
    }

    /**
     * To pass an obstacle, the robot's hood must be stowed and the state must be
     * Position. This function checks that the robot is safe to pass through an
     * obstacle.
     * 
     * @return If the robot is safe to pass through an obstacle
     */
    public synchronized boolean isSafe() {
        return (control_mode_ == ControlMode.POSITION && getAngle().getDegrees() < Constants.Hood.HOOD_MAX_SAFE_ANGLE
                && pidf_.getSetpoint() < Constants.Hood.HOOD_MAX_SAFE_ANGLE);
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("has_hood_homed", has_homed_);
        SmartDashboard.putNumber("hood_angle", getAngle().getUnboundedDegrees());
        SmartDashboard.putNumber("hood_setpoint", pidf_.getSetpoint());
        SmartDashboard.putBoolean("hood_on_target", isOnTarget());
        SmartDashboard.putNumber("hood_error", pidf_.getSetpoint() - getAngle().getDegrees());
    }

    @Override
    public synchronized void stop() {
        pidf_.reset();
        control_mode_ = ControlMode.OPEN_LOOP;
        set(0);
    }

    @Override
    public synchronized void zeroSensors() {
        // encoder_.reset();
        // encoder_.setPosition(0,50);
    }

    public boolean isStowed() {
        return false;
    }

    void setStowed(boolean stow) {

    }

    public void setTestServoSpeed(double speed) {
        // test_servo_.set(speed);
    }

    @Override
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(loop);
	}


}