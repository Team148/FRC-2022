package frc;

import java.util.Arrays;
import java.util.List;

import com.team1323.lib.util.InterpolatingDouble;
import com.team1323.lib.util.InterpolatingTreeMap;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

public class Constants {
	/*All distance measurements are in inches, unless otherwise noted.*/

	//Loop Constants
	public static final double kLooperDt = 0.02;
	public static final int kCANTimeoutMs = 10; //use for on the fly updates
    public static final int kLongCANTimeoutMs = 100; //use for constructors
	public static final double kEpsilon = 0.0001;
	
	//Settings
	public static final boolean kIsUsingCompBot = true;
	public static final boolean kIsUsingTractionWheels = true;
	public static final boolean kDebuggingOutput = true;
	public static final boolean kResetTalons = false;

	//Physical Robot Dimensions (including bumpers)
	public static final double kRobotWidth = 33.16; // Waco 34.5;
	public static final double kRobotLength = 33.16;  // Waco 34.5;
	public static final double kRobotHalfWidth = kRobotWidth / 2.0;
	public static final double kRobotHalfLength = kRobotLength / 2.0;

	/**
    * Target Specifications
    */
    public static final double kVisionTargetHeight = 102.5; //Tweaked for 2022 Rapid React 
    public static final Rotation2d kPortTargetOrientation = Rotation2d.fromDegrees(0.0);
    public static final Translation2d kOuterPortToInnerPort = new Translation2d(29.25, 0.0);
	public static final double kGoalTargetDiameter = 53.0;

	//Swerve Calculations Constants (measurements are in inches)
    public static final double kWheelbaseLength = 21.25; //Tweaked for Quickdraw //was 20.5 for Waco
    public static final double kWheelbaseWidth  = 21.25; //Tweaked for Quickdraw //was 20.5 for Waco
    public static final double kSwerveDiagonal = Math.hypot(kWheelbaseLength, kWheelbaseWidth);
    
    //Camera Constants (X and Y are with respect to the turret's center)
    public static final double kCameraYOffset = 0.0; //Tweaked for Quickdraw
    public static final double kCameraXOffset = -6.5; // Tweaked for Quickdraw
    public static final double kCameraZOffset = 36.67; // Tweaked for Quickdraw
    public static final double kCameraYawAngleDegrees = 0; //-2.25;//-12.7
    public static final double kCameraPitchAngleDegrees = Settings.kIsUsingCompBot ? 24.0 : 24.0; //Tweaked for Quickdraw -- 32.0 / 32.0

    //Limelight
    public static final double kHorizontalFOV = 59.6; // degrees
    public static final double kVerticalFOV = 49.7; // degrees
    public static final double kVPW = 2.0 * Math.tan(Math.toRadians(kHorizontalFOV / 2.0));
    public static final double kVPH = 2.0 * Math.tan(Math.toRadians(kVerticalFOV / 2.0));
    public static final double kImageCaptureLatency = 11.0 / 1000.0; // seconds
    
    //Goal tracker constants
    public static final double kMaxGoalTrackAge = 2.5;
    public static final double kMaxTrackerDistance = 18.0;
    public static final double kCameraFrameRate = 90.0;
    public static final double kTrackStabilityWeight = 10.0;
    public static final double kTrackAgeWeight = 5.0;
    public static final double kTrackSwitchingWeight = 10.0;
	public static final double kClosestVisionDistance = 26.0;//36.0

	public static final double kVisionPIDOutputPercent = 0.5;
    public static final double kPosePredictionTime = 1.3; //0.125; // seconds 0.25 //GC changed from 0.75 3/12
	
	public static final double kDistanceToTargetTolerance = 1.0;
    public static final double kGyroDriftPerRotation = -0.25; // degrees

	//joystick constants
	public static final double kDriverTriggerOFFMIN = 0;
	public static final double kDriverTriggerOFFMAX = 0.1;

	public static final double kDriverTriggerHalfMIN = 0.2;
	public static final double kDriverTriggerHalfMAX = 0.7;
	
	public static final double kDriverTriggerFullMIN = 0.8;
	public static final double kDriverTriggerFullMAX = 1.0;

	//Color see ball oooo
	public static final double kRedBallHMax = 360.0;
	public static final double kRedBallHMin = 310.0;
	public static final double kBlueBallHMax = 210.0;
	public static final double kBlueBallHMin = 120.0;

	public static final double kIRThreshold = 20.0;

	public static final double kBallTimeThreshold = 0.05;

	public static final double speedBuff = 1.06; // practice bot is 1.0; comp bot is 1.06
	// Practice bot has + .03, .03, .04, .05 on top four speedbuffs

	//Vision Speed Constraint Treemap -- Units are inches to center of goal, then ticks per 100ms (Falcon Units)
	public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kVisionSpeedTreemap = new InterpolatingTreeMap<>();
		static{
			kVisionSpeedTreemap.put(new InterpolatingDouble(64.0+20.0), new InterpolatingDouble(10750.0*speedBuff));
			// kVisionSpeedTreemap.put(new InterpolatingDouble(kClosestVisionDistance), new InterpolatingDouble(24.0));
			kVisionSpeedTreemap.put(new InterpolatingDouble(95.0+20.0+25.0), new InterpolatingDouble(11250.0*speedBuff));
			kVisionSpeedTreemap.put(new InterpolatingDouble(132.0+20.0+25.0), new InterpolatingDouble(11750.0*speedBuff));
			kVisionSpeedTreemap.put(new InterpolatingDouble(167.0+20.0+25.0), new InterpolatingDouble(12750.0*(speedBuff))); //speedbuff + 0.03
			kVisionSpeedTreemap.put(new InterpolatingDouble(195.0+20.0+25.0), new InterpolatingDouble(13500.0*(speedBuff+0.06))); //3
			kVisionSpeedTreemap.put(new InterpolatingDouble(228.0+20.0+25.0 ), new InterpolatingDouble(14500.0*(speedBuff+0.06)));//4
			kVisionSpeedTreemap.put(new InterpolatingDouble(287.0+25.0), new InterpolatingDouble(16250.0*(speedBuff)));      //5
		}

	//Vision Angle Constraint Treemap - Units are inches to center of goal / hood angle in degrees
	public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kVisionAngleTreemap = new InterpolatingTreeMap<>();
		static{
			kVisionAngleTreemap.put(new InterpolatingDouble(64.0+20.0), new InterpolatingDouble(14.0));
			kVisionAngleTreemap.put(new InterpolatingDouble(95.0+20.0+25.0), new InterpolatingDouble(27.0));
			kVisionAngleTreemap.put(new InterpolatingDouble(132.0+20.0+25.0), new InterpolatingDouble(33.0));
			kVisionAngleTreemap.put(new InterpolatingDouble(167.0+20.0+25.0), new InterpolatingDouble(36.0));
			kVisionAngleTreemap.put(new InterpolatingDouble(195.0+20.0+25.0), new InterpolatingDouble(38.0));
			kVisionAngleTreemap.put(new InterpolatingDouble(228.0+20.0+25.0), new InterpolatingDouble(44.0));
			kVisionAngleTreemap.put(new InterpolatingDouble(287.0+25.0), new InterpolatingDouble(44.0));

		}

	//Vision Time of Flight Map -- Units are inches to center of goal / seconds; currently based off video from lab
	public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kVisionToFTreemap = new InterpolatingTreeMap<>();
		static{
			kVisionToFTreemap.put(new InterpolatingDouble(64.0+20.0), new InterpolatingDouble(0.7));
			// kVisionToFTreemap.put(new InterpolatingDouble(95.0+20.0), new InterpolatingDouble(1.0));
			// kVisionToFTreemap.put(new InterpolatingDouble(132.0+20.0), new InterpolatingDouble(1.25));
			// kVisionToFTreemap.put(new InterpolatingDouble(167.0), new InterpolatingDouble(1.3)); //made up
			// kVisionToFTreemap.put(new InterpolatingDouble(195.0), new InterpolatingDouble(1.30)); //made up
			// kVisionToFTreemap.put(new InterpolatingDouble(228.0+20.0), new InterpolatingDouble(1.4));
			kVisionToFTreemap.put(new InterpolatingDouble(287.0), new InterpolatingDouble(1.2)); //made up // changed by GC to 1.2
		}

	//Vision Limelight Distance Calibration Treemap -- Janky fix to make sure we get good coorilation between tape measure and LL
	public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kVisionDistanceTreemap = new InterpolatingTreeMap<>();
		static{
			/* =================== THE PROCEDURE FOR THIS: =================== 
			- Put the end of the special tape measure against the hub
			- Move the robot back such that the front bumper lines up with each line
			- Fill in "Camera Distance" from Shuffleboard under the following values:
			
			64 = 
			95 = 
			132 = 
			167 = 
			195 = 
			228 = 
			252 = 
			287 = 

			- These new values go in the first column of the InterpolatingDouble table below
			*/ 

			//Practice Bot
			// kVisionDistanceTreemap.put(new InterpolatingDouble(67.0), new InterpolatingDouble(64.0));
			// kVisionDistanceTreemap.put(new InterpolatingDouble(93.0), new InterpolatingDouble(95.0));
			// kVisionDistanceTreemap.put(new InterpolatingDouble(129.0), new InterpolatingDouble(132.0));
			// kVisionDistanceTreemap.put(new InterpolatingDouble(159.0), new InterpolatingDouble(167.0));
			// kVisionDistanceTreemap.put(new InterpolatingDouble(181.0), new InterpolatingDouble(195.0));
			// kVisionDistanceTreemap.put(new InterpolatingDouble(215.0), new InterpolatingDouble(228.0));
			// kVisionDistanceTreemap.put(new InterpolatingDouble(234.0), new InterpolatingDouble(252.0));
			// kVisionDistanceTreemap.put(new InterpolatingDouble(259.0), new InterpolatingDouble(287.0));

			//Comp Bot
			// kVisionDistanceTreemap.put(new InterpolatingDouble(80.0), new InterpolatingDouble(64.0));
			// kVisionDistanceTreemap.put(new InterpolatingDouble(115.0), new InterpolatingDouble(95.0));
			// kVisionDistanceTreemap.put(new InterpolatingDouble(162.0), new InterpolatingDouble(132.0));
			// kVisionDistanceTreemap.put(new InterpolatingDouble(207.0), new InterpolatingDouble(167.0)); //reads 213
			// kVisionDistanceTreemap.put(new InterpolatingDouble(239.0), new InterpolatingDouble(195.0)); //reads 243
			// kVisionDistanceTreemap.put(new InterpolatingDouble(302.0), new InterpolatingDouble(228.0)); //reads 296
			// kVisionDistanceTreemap.put(new InterpolatingDouble(333.0), new InterpolatingDouble(252.0)); //reads 321
			// kVisionDistanceTreemap.put(new InterpolatingDouble(382.0), new InterpolatingDouble(287.0)); //reads 372
			
			//Comp Bot in shop on 4-16
			kVisionDistanceTreemap.put(new InterpolatingDouble(80.5), new InterpolatingDouble(64.0));
			kVisionDistanceTreemap.put(new InterpolatingDouble(114.1), new InterpolatingDouble(95.0));
			kVisionDistanceTreemap.put(new InterpolatingDouble(159.0), new InterpolatingDouble(132.0));
			kVisionDistanceTreemap.put(new InterpolatingDouble(206.0), new InterpolatingDouble(167.0));
			kVisionDistanceTreemap.put(new InterpolatingDouble(239.0), new InterpolatingDouble(195.0)); 
			kVisionDistanceTreemap.put(new InterpolatingDouble(299.5), new InterpolatingDouble(228.0)); 
			kVisionDistanceTreemap.put(new InterpolatingDouble(330.0), new InterpolatingDouble(252.0)); 
			kVisionDistanceTreemap.put(new InterpolatingDouble(380.0), new InterpolatingDouble(287.0)); 

			//Practice LL on CompBot 0418
			// kVisionDistanceTreemap.put(new InterpolatingDouble(77.5), new InterpolatingDouble(64.0));
			// kVisionDistanceTreemap.put(new InterpolatingDouble(105.0), new InterpolatingDouble(95.0));
			// kVisionDistanceTreemap.put(new InterpolatingDouble(148.0), new InterpolatingDouble(132.0));
			// kVisionDistanceTreemap.put(new InterpolatingDouble(187.0), new InterpolatingDouble(167.0));
			// kVisionDistanceTreemap.put(new InterpolatingDouble(216.0), new InterpolatingDouble(195.0)); 
			// kVisionDistanceTreemap.put(new InterpolatingDouble(265.5), new InterpolatingDouble(228.0)); 
			// kVisionDistanceTreemap.put(new InterpolatingDouble(292.0), new InterpolatingDouble(252.0)); 
			// kVisionDistanceTreemap.put(new InterpolatingDouble(333.5), new InterpolatingDouble(287.0)); 


			//Comp Bot - Irving
			// kVisionDistanceTreemap.put(new InterpolatingDouble(82.3), new InterpolatingDouble(64.0+0.0)); // bounces between 81.4 83.2
			// kVisionDistanceTreemap.put(new InterpolatingDouble(116.3), new InterpolatingDouble(95.0+7.0)); //115.6 - 117
			// kVisionDistanceTreemap.put(new InterpolatingDouble(160.9), new InterpolatingDouble(132.0+10.0)); //160.9 - (goal range report 151)
			// kVisionDistanceTreemap.put(new InterpolatingDouble(209.5), new InterpolatingDouble(167.0+10.0)); //208 - 211
			// kVisionDistanceTreemap.put(new InterpolatingDouble(236.6), new InterpolatingDouble(195.0+10.0)); //246 - 247
			// kVisionDistanceTreemap.put(new InterpolatingDouble(293.5), new InterpolatingDouble(228.0+0.0)); //293 - 294
			// kVisionDistanceTreemap.put(new InterpolatingDouble(330.5), new InterpolatingDouble(252.0+0.0)); //330.56 (reported 270.13)
			// kVisionDistanceTreemap.put(new InterpolatingDouble(389.0), new InterpolatingDouble(287.0+0.0)); //388.98 
		}
    
    //Path following constants
    public static final double kPathLookaheadTime = 0.25;  // seconds to look ahead along the path for steering 0.4
	public static final double kPathMinLookaheadDistance = 6.0;  // inches 24.0 (we've been using 3.0)
    
    //Swerve Speed Constants
    public static final double kSwerveDriveMaxSpeed = 22000.0;
    public static final double kSwerveMaxSpeedInchesPerSecond = 200;//12.5 * 12.0;
	public static final double kSwerveRotationMaxSpeed = 12720.0 * 0.8; //The 0.8 is to request a speed that is always achievable
	public static final double kSwerveRotation10VoltMaxSpeed = 1350.0;
    public static final double kSwerveRotationSpeedScalar = ((1.0 / 0.125) - 1.0) / kSwerveMaxSpeedInchesPerSecond;
    
	//Swerve Module Wheel Offsets (Rotation encoder values when the wheels are facing 0 degrees)
	/**
    * To Zero: Rotate module so that bevel gear is face out. Rotate module 90° CW from the top
    * Enter angle read by the absolute encoder. Insert as degrees and subtract or add 90° to the value
    * based on where the bevel ended up.
    */
	public static final double kFrontRightEncoderStartingPos = kIsUsingCompBot ? 0 : 0;//-350 : 0;//-176
	public static final double kFrontLeftEncoderStartingPos = kIsUsingCompBot ? 0 : 0;//-31 : 0;//-343
	public static final double kRearLeftEncoderStartingPos = kIsUsingCompBot ? 0 : 0;//-358 : 0;//-90
	public static final double kRearRightEncoderStartingPos = kIsUsingCompBot ? 0 : 0;//-145 : 0;//-271 
	
	//Swerve Module Positions (relative to the center of the drive base)
	public static final Translation2d kVehicleToModuleZero = new Translation2d(kWheelbaseLength/2, kWheelbaseWidth/2);
	public static final Translation2d kVehicleToModuleOne = new Translation2d(kWheelbaseLength/2, -kWheelbaseWidth/2);
	public static final Translation2d kVehicleToModuleTwo = new Translation2d(-kWheelbaseLength/2, -kWheelbaseWidth/2);
	public static final Translation2d kVehicleToModuleThree = new Translation2d(-kWheelbaseLength/2, kWheelbaseWidth/2);
	
	public static final List<Translation2d> kModulePositions = Arrays.asList(kVehicleToModuleZero,
			kVehicleToModuleOne, kVehicleToModuleTwo, kVehicleToModuleThree);
	
	//Scrub Factors
	public static final boolean kSimulateReversedCarpet = false;
	public static final double[] kWheelScrubFactors = new double[]{1.0, 1.0, 1.0, 1.0};
	public static final double kXScrubFactor = 1.0;//1.0 / (1.0 - (9549.0 / 293093.0));
	public static final double kYScrubFactor = 1.0;//1.0 / (1.0 - (4.4736 / 119.9336));

	//Voltage-Velocity equation constants {m, b, x-intercept}
	//First set is the positive direction, second set is negative
	public static final double[][][] kVoltageVelocityEquations = new double[][][]{
		{{1.70, -4.39, 2.58}, {1.83, 5.23, -2.85}}, 
		{{1.59, -3.86, 2.42}, {1.43, 3.09, -2.16}}, 
		{{1.53, -3.66, 2.39}, {1.66, 4.15, -2.50}}, 
		{{1.84, -4.70, 2.56}, {1.85, 5.34, -2.89}}};
	
	//Swerve Odometry Constants
	public static final double kSwerveWheelDiameter = 3.96; //inches (actual diamter is closer to 3.87, but secondary algorithm prefers 4.0901) 3.76
    public static final double kSwerveDriveEncoderResolution = 2048.0; //2048.0 for falcon 500
	public static final double kSwerveRotationEncoderResolution = 2048.0;
	  /** The number of rotations the swerve rotation motor undergoes for every rotation of the module. */
	  public static final double kSwerveRotationReduction = (72.0 * 24.0) / (14.0 * 12.0 );//12.0;
	/** The number of rotations the swerve drive encoder undergoes for every rotation of the wheel. */
	public static final double kSwerveEncoderToWheelRatio = 6.54;
	public static final double kSwerveEncUnitsPerWheelRev = kSwerveDriveEncoderResolution * kSwerveEncoderToWheelRatio;
	public static final double kSwerveEncUnitsPerInch = kSwerveEncUnitsPerWheelRev / (Math.PI * kSwerveWheelDiameter);

	public static class Feeder {

		//Feeder Constants
		public static final double FEEDER_SHOOTING_SPEED = 0.75;
		public static final double FEEDER_INTAKE_SPEED = 0.3;
		public static final double FEEDER_UNJAM_SPEED = -0.5;
		public static final double HOPPER_UNJAM_SPEED = 0.67;

		//Hopper Constants
		public static final double HOPPER_INTAKING_SPEED = 0.2;
		public static final double HOPPER_SHOOTING_SPEED = -0.7;
		public static final double HOPPER_WOF_SPEED = 1.0;

	}

	public static class BallIntake {

		//Ball Intake Constants
		public static final double INTAKING_SPEED = -1.0;
		public static final double OUTTAKING_SPEED = 1.0;
		
	}

	public static class Hanger {

		//Hanger Constants
		public static final int HANGER_DOWN_SLOW = -11000;
	
	}

	public static class Leds {

		//LED Colors
		public static final List<Double> PINK = Arrays.asList(255.0, 20.0, 30.0);
		public static final List<Double> BLUE = Arrays.asList(0.0, 0.0, 255.0);
		public static final List<Double> RED = Arrays.asList(255.0, 0.0, 0.0);
		public static final List<Double> ORANGE = Arrays.asList(255.0, 20.0, 0.0);
		public static final List<Double> YELLOW = Arrays.asList(255.0, 60.0, 0.0);
		public static final List<Double> GREEN = Arrays.asList(0.0, 255.0, 0.0);
		public static final List<Double> PURPLE = Arrays.asList(255.0, 0.0, 255.0);

		//LED Arrays
		public static final List<List<Double>> rainbow = Arrays.asList(RED, ORANGE, YELLOW, GREEN, BLUE, PINK, PURPLE);

	}

	public static class Shooter {

		//Shooter Gains
		public static double FLYWHEEL_KP = 0.06;//0.12;
		public static double FLYWHEEL_KI = 0.0; //0.0
		public static double FLYWHEEL_KD = 0.2;//0.5;
		public static double FLYWHEEL_KF = 0.057;//0.014;
		public static int FLYWHEEL_IZONE = (int) (1023.0 / FLYWHEEL_KP);
		public static double FLYWHEEL_RAMPRATE = 0;
		public static int FLYWHEEL_ALLOWED_ERROR = 0;

		//Shooter Constants
		public static double SHOOTER_VELOCITY_TOLERANCE = 260;

		public static double COMP_FAR_FLYWHEEL = 13500;

		public static double MINIMUM_SHOOTER_SPEED = 0;

		public static double AT_GOAL = 10750;//11500.0;//incorrect
		public static double BACK_LINE = 11500.0;//11200.0;
		public static double LAUNCH_PAD = 12500.0;
		public static double FAR_LAUNCH_PAD = 13200.0;//incorrect
		public static double HP_WALL = 16250.0;//13000.0;//incorrect

	}

	public static class Hood {

		//Hood Gains
		public static double HOOD_KP = 0.1;
		public static double HOOD_KI = 0.0;
		public static double HOOD_KD = 0.0;

		//Hood Constants
		public static double SAFE_HOOD_ANGLE = 0.0;
		public static double SAFE_HOOD_SETPOINT = 22.0;
		public static double HOOD_MAX_SAFE_ANGLE = 15.0;
		public static double MIN_HOOD_ANGLE = 5.0;
		public static double MAX_HOOD_ANGLE = 57.0;
		public static double HOOD_ON_TARGET_TO_TOLERANCE = 0.5;
		public static double HOOD_DEADBAND = 0.3; // degrees
		public static double HOOD_GEAR_REDUCTION =  (36.0/14.0)*(24.0/364.0);// (18.0/364.0);	//1.0;
		public static double HOOD_ENCODER_OFFSET = (-44.0 - 37.0);	//Rollover Prevention -zero this out, read in Phoenix, then calibrate this number to add to 30*
		public static double HOOD_ANGLE_OFFSET = -30.0;
		public static double HOOD_SHOOTING_DEGREE_TOLERANCE = 3.0; // degrees

		public static double SAFE_TURRET_ANGLE_FORWARD = 0.0;
		public static double SAFE_TURRET_ANGLE_BACKWARD_POSITIVE = 180.0;
		public static double SAFE_TURRET_ANGLE_BACKWARD_NEGATIVE = -180.0;
		public static double SAFE_TURRET_TOLERANCE = 15.0;

	}

	public static class Turret {

		//Turret Gains
		public static double TURRET_KP = 0.3; // 0.7;
		public static double TURRET_KI = 0.0;
		public static double TURRET_KD = 0.0;// 0.0;
		public static double TURRET_KF = 0.0465;
		public static int TURRET_IZONE = (int) (1023.0 / TURRET_KP);
		public static double TURRET_RAMPRATE = 0;
		public static int TURRET_ALLOWED_ERROR = 100;

		//Turret Constants
		public static final double TURRET_MAXSPEED = 22000.0;//22000.0;

		// public static final double TURRET_MAXCONTROLANGLE = 325.0;//305.0;
		// public static final double TURRET_MINCONTROLANGLE = 15.0;//35.0;
		// public static final double TURRET_MAXINITIALANGLE = 325.0;//305.0;
		// public static final double TURRET_MININITIALLANGLE = 15.0;//35.0;

		public static final double TURRET_MAXCONTROLANGLE = 55.0;//305.0;
		public static final double TURRET_MINCONTROLANGLE = -255.0;//35.0;
		public static final double TURRET_MAXINITIALANGLE = 55.0;//305.0;
		public static final double TURRET_MININITIALLANGLE = -255.0;//35.0;

		//Turret pose with respect to the robot's center
		public static final double kXOffset = -5.50; //Tweaked for 2022 Rapid React - Quickdraw
		public static final double kYOffset = -5.50; //Tweaked for 2022 Rapid React - Quickdraw

		public static final double kEncoderStartingAngle = Settings.kIsUsingCompBot ? -180 : -180;//90.0 : 0.0; //40.2 : -110.5; // Absolute position of the magnet 309.25
		public static final double kAngleTolerance = 1.0;

		//Ratios
		public static final double kInternalEncToOutputRatio = ((140.0 * 40.0) / (10.0 * 8.0));//((148.0 * 50.0) / (8.0 * 12.0));//((213 * 50) / (8 * 18));//100.0;
		public static final double kEncoderToOutputRatio = 14.0;//1.0;

		public static final List<double[]> kVisionRanges = Arrays.asList(
			new double[] {TURRET_MINCONTROLANGLE, TURRET_MAXCONTROLANGLE},
			new double[] {TURRET_MAXCONTROLANGLE, TURRET_MAXCONTROLANGLE}
		);
	}

	public static class IntakePivot {

		//IntakePivot Gains
		public static double INTAKEPIVOT_KP = kIsUsingCompBot ? 0.075 : 0.0375;
		public static double INTAKEPIVOT_KI = 0.0;
		public static double INTAKEPIVOT_KD = 0.0;// 0.0;
		public static double INTAKEPIVOT_KF = 0.0;
		public static int INTAKEPIVOT_IZONE = (int) (1023.0 / INTAKEPIVOT_KP);
		public static double INTAKEPIVOT_RAMPRATE = 0;
		public static int INTAKEPIVOT_ALLOWED_ERROR = 100;

		//IntakePivot Constants
		public static final double INTAKEPIVOT_MAXSPEED = 22000.0;//10000.0;

		public static final double INTAKEPIVOT_UP = -30700.0;
		public static final double INTAKEPIVOT_DOWN = -47800.0;

		public static final double INTAKEPIVOT_ZEROEDANGLE = 0.0;
		public static final double INTAKEPIVOT_MAXCONTROLANGLE = kIsUsingCompBot ? 135.0 : 103.0;//In both positive and negative directions | 220.0
		public static final double INTAKEPIVOT_MINCONTROLANGLE = 45.0; // -25.0
		public static final double INTAKEPIVOT_SNATCHANGLE = 100.0;


		public static final double kEncoderStartingAngle = Settings.kIsUsingCompBot ? 0.0 : 0.0;
		public static final double kAngleTolerance = 1.0;

		//Ratios
		public static final double kInternalEncToOutputRatio = kIsUsingCompBot ? ((54.0 * 60.0) / (10.0 * 12.0)) : ((54.0 * 84.0 *  48.0) / (8.0 * 30.0 * 16.0));

	}

	public static class MotorizedHood {

		//Hood Gains
		public static double kP = 0.1;
		public static double kI = 0.0;
		public static double kD = 0.0;

		public static double kAngleDeadband = 0.3; // degrees

		public static final double kEncoderRatio = (355.0/18) / (36.0/10);

		// public static final double kReduction = 80.89;

		public static final double kEncoderOffset = Settings.kIsUsingCompBot? (-177.0): (-177.0);	//Rollover Prevention -zero this out, read in Phoenix, then calibrate this number to add to 30*

		public static final double kHoodStartingAngle = 65.0;
        public static final double kEncStartingAngle = Settings.kIsUsingCompBot ? -249.1 : -235.1; // The absolute angle (in degrees) of the mag encoder when the hood is at kHoodStartingAngle

		public static final double kMinInitialAngle = 7.0;
		// public static final double kMaxInitialAngle = 70.0;
		
		public static final double kMinControlAngle = Settings.kIsUsingCompBot ? 7.0 : 7.0;
        public static final double kMaxControlAngle = 52.0;

		public static final double kAngleTolerance = 2.0;
		
		//Shooter Hood Angles
		public static final double CompCloseAngle = 7.0;//32.25;
		public static final double CompMidAngle = 20.5; //55 deg
		public static final double CompFarAngle = 55.0;//59.0;
		public static final double R2CloseAngle = 7.0;//32.25;
		public static final double R2MidAngle = 20.5; //55 deg
		public static final double R2FarAngle = 49.0;//59.0;
		
	}

	public static class FalconHood{

		public static double kP = 0.2;
		public static double kI = 0.0;
		public static double kD = 0.0;

		public static final double kEncoderRatio =  (52 * 460) / (8 * 14); //no longer pending

		public static final double kMinControlAngle = 0.0;
		public static final double kMaxControlAngle = 48.0;

		public static final double kAngleTolerance = 2.0;

		public static final double AT_GOAL = 7.0;
		public static final double BACK_LINE = 28.0;
		public static final double LAUNCH_PAD = 41.0;
		public static final double HP_WALL = 46.0;
	}

	//Field Landmarks
	public static final double kFieldLength = 625.0;
	public static final double kFieldHalfWidth = 163.0;

	//Field Positions
	// 135.0 inches from full field to start line pose
	// public static final Pose2d twoBallStart = new Pose2d(new Translation2d(297.0, -51.0), Rotation2d.fromDegrees(0.0)); //250.0
	public static final Pose2d twoBallStart = new Pose2d(new Translation2d(kFieldLength - 290.0, -90.0), Rotation2d.fromDegrees(0.0));
	public static final Pose2d twoBallOne = new Pose2d(new Translation2d(kFieldLength - 290.0, -127.0), Rotation2d.fromDegrees(-90.0));
	public static final Pose2d twoBallOne2 = new Pose2d(new Translation2d(kFieldLength - 290.0, -127.0), Rotation2d.fromDegrees(20.0));
	public static final Pose2d twoBallTwo = new Pose2d(new Translation2d(kFieldLength - 195.0, -100.0), Rotation2d.fromDegrees(20.0));
	public static final Pose2d twoBallTutu = new Pose2d(new Translation2d(kFieldLength - 195.0, -100.0), Rotation2d.fromDegrees(-8.0));
	public static final Pose2d twoBallTerminal = new Pose2d(new Translation2d(kFieldLength - 43.0, -100.0), Rotation2d.fromDegrees(-45.0)); //x was 40 y was -95
	public static final Pose2d twoBallTerminal2 = new Pose2d(new Translation2d(kFieldLength - 43.0, -100.0), Rotation2d.fromDegrees(184.0)); //x was 40 y was -95
	public static final Pose2d postTerminalShot = new Pose2d(new Translation2d(kFieldLength - 312.0 , -120.0), Rotation2d.fromDegrees(184.0));

	public static final Pose2d oneBallStart = new Pose2d(new Translation2d(kFieldLength - 250.0, 53.0), Rotation2d.fromDegrees(0.0));
	public static final Pose2d oneBallOne = new Pose2d(new Translation2d(kFieldLength - 195.0, 88.0), Rotation2d.fromDegrees(45.0));
	public static final Pose2d oneBallOne2 = new Pose2d(new Translation2d(kFieldLength - 195.0, 88.0), Rotation2d.fromDegrees(-90.0));
	public static final Pose2d oneBallOne3 = new Pose2d(new Translation2d(kFieldLength - 195.0, 90.0), Rotation2d.fromDegrees(0.0));

	public static final Pose2d oneBallOneDefend = new Pose2d(new Translation2d(kFieldLength - 160.0, -30.0), Rotation2d.fromDegrees(-90.0));
	public static final Pose2d oneBallOneDefend2 = new Pose2d(new Translation2d(kFieldLength - 160.0, -20.0), Rotation2d.fromDegrees(135.0));
	public static final Pose2d oneBallToDefend = new Pose2d(new Translation2d(kFieldLength - 215.0, 117.0), Rotation2d.fromDegrees(135.0));
	public static final Pose2d oneBallToDefend2 = new Pose2d(new Translation2d(kFieldLength - 215.0, 117.0), Rotation2d.fromDegrees(135.0));
	public static final Pose2d oneBallDefendDropOff = new Pose2d(new Translation2d(kFieldLength - 210.0, 30.0), Rotation2d.fromDegrees(0.0));

	// public static final Pose2d oneBallOneDefendPA = new Pose2d(new Translation2d(kFieldLength - 160.0, -30.0), Rotation2d.fromDegrees(-90.0));
	// public static final Pose2d oneBallOneDefendPA2 = new Pose2d(new Translation2d(kFieldLength - 160.0, -30.0), Rotation2d.fromDegrees(135.0));
	// public static final Pose2d oneBallToDefendPA = new Pose2d(new Translation2d(kFieldLength - 215.0, 117.0), Rotation2d.fromDegrees(135.0));

	public static final Pose2d oneBallTerminal = new Pose2d(new Translation2d(kFieldLength - 40.0, -95.0), Rotation2d.fromDegrees(-45.0));
	public static final Pose2d oneBallTerminalShot = new Pose2d(new Translation2d(kFieldLength - 150.0, 0.0), Rotation2d.fromDegrees(0.0));

	public static final Pose2d oneBallAndPartnerBalls = new Pose2d(new Translation2d(kFieldLength - 225.0, 33.0), Rotation2d.fromDegrees(-90.0));
	public static final Pose2d oneBallAndPartnerBalls2 = new Pose2d(new Translation2d(kFieldLength - 225.0, 33.0), Rotation2d.fromDegrees(90.0));



	public static final Pose2d swerveReset = new Pose2d(new Translation2d(0.0 , 0.0), Rotation2d.fromDegrees(180.0));

}
