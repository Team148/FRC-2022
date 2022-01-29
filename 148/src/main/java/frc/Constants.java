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
	public static final double kRobotWidth = 34.5;//36.5;
	public static final double kRobotLength = 34.5;//36.5;
	public static final double kRobotHalfWidth = kRobotWidth / 2.0;
	public static final double kRobotHalfLength = kRobotLength / 2.0;

	/**
    * Target Specifications
    */
    public static final double kVisionTargetHeight = 98.25; //81.0 to bottom
    public static final Rotation2d kPortTargetOrientation = Rotation2d.fromDegrees(0.0);
    public static final Translation2d kOuterPortToInnerPort = new Translation2d(29.25, 0.0);

	//Swerve Calculations Constants (measurements are in inches)
    public static final double kWheelbaseLength = 20.5; //16.5
    public static final double kWheelbaseWidth  = 20.5; //16.5
    public static final double kSwerveDiagonal = Math.hypot(kWheelbaseLength, kWheelbaseWidth);
    
    //Camera Constants (X and Y are with respect to the turret's center)
    public static final double kCameraYOffset = 0.0;//0.25
    public static final double kCameraXOffset = 9.13;//8.216; //8.5
    public static final double kCameraZOffset = 26.48;//25.0; //26.776 24.524
    public static final double kCameraYawAngleDegrees = -2.25;//-12.7
    public static final double kCameraPitchAngleDegrees = Settings.kIsUsingCompBot ? 30.0 : 30.0; //21.75 for bottom 34.3 37.0604

    //Limelight
    public static final double kHorizontalFOV = 59.6; // degrees
    public static final double kVerticalFOV = 49.7; // degrees
    public static final double kVPW = 2.0 * Math.tan(Math.toRadians(kHorizontalFOV / 2.0));
    public static final double kVPH = 2.0 * Math.tan(Math.toRadians(kVerticalFOV / 2.0));
    public static final double kImageCaptureLatency = 11.0 / 1000.0; // seconds
    
    //Goal tracker constants
    public static final double kMaxGoalTrackAge = 0.5;
    public static final double kMaxTrackerDistance = 18.0;
    public static final double kCameraFrameRate = 90.0;
    public static final double kTrackStabilityWeight = 1.0;
    public static final double kTrackAgeWeight = 1.0;
    public static final double kTrackSwitchingWeight = 0.0;
	public static final double kClosestVisionDistance = 26.0;//36.0

	public static final double kVisionPIDOutputPercent = 0.5;
    public static final double kPosePredictionTime = 0.125; // seconds 0.25
	
	public static final double kDistanceToTargetTolerance = 1.0;
    public static final double kGyroDriftPerRotation = -0.25; // degrees

	//Vision Speed Constraint Treemap
	public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kVisionSpeedTreemap = new InterpolatingTreeMap<>();
	static{
		kVisionSpeedTreemap.put(new InterpolatingDouble(-6.0), new InterpolatingDouble(24.0));
		kVisionSpeedTreemap.put(new InterpolatingDouble(kClosestVisionDistance), new InterpolatingDouble(24.0));
		kVisionSpeedTreemap.put(new InterpolatingDouble(60.0), new InterpolatingDouble(48.0));
		kVisionSpeedTreemap.put(new InterpolatingDouble(300.0), new InterpolatingDouble(48.0));
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
	public static final double kFrontRightEncoderStartingPos = kIsUsingCompBot ? -350 : 0;//-176
	public static final double kFrontLeftEncoderStartingPos = kIsUsingCompBot ? -31 : 0;//-343
	public static final double kRearLeftEncoderStartingPos = kIsUsingCompBot ? -358 : 0;//-90
	public static final double kRearRightEncoderStartingPos = kIsUsingCompBot ? -145 : 0;//-271 
	
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
	  public static final double kSwerveRotationReduction = 12.0;
	/** The number of rotations the swerve drive encoder undergoes for every rotation of the wheel. */
	public static final double kSwerveEncoderToWheelRatio = 6.54;
	public static final double kSwerveEncUnitsPerWheelRev = kSwerveDriveEncoderResolution * kSwerveEncoderToWheelRatio;
	public static final double kSwerveEncUnitsPerInch = kSwerveEncUnitsPerWheelRev / (Math.PI * kSwerveWheelDiameter);

	public static class Feeder {

		//Feeder Constants
		public static final double FEEDER_SHOOTING_SPEED = 0.75;
		public static final double FEEDER_UNJAM_SPEED = -0.5;
		public static final double HOPPER_UNJAM_SPEED = 0.67;

		//Hopper Constants
		public static final double HOPPER_INTAKING_SPEED = 0.2;
		public static final double HOPPER_SHOOTING_SPEED = -0.7;
		public static final double HOPPER_WOF_SPEED = 1.0;

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
		public static double FLYWHEEL_KP = 0.12;
		public static double FLYWHEEL_KI = 0.0;
		public static double FLYWHEEL_KD = 0.5;
		public static double FLYWHEEL_KF = 0.014;
		public static int FLYWHEEL_IZONE = (int) (1023.0 / FLYWHEEL_KP);
		public static double FLYWHEEL_RAMPRATE = 0;
		public static int FLYWHEEL_ALLOWED_ERROR = 0;

		//Shooter Constants
		public static double SHOOTER_VELOCITY_TOLERANCE = 260;

		public static double COMP_FAR_FLYWHEEL = 13500;

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

		public static final double TURRET_MAXCONTROLANGLE = 210.0; //In both positive and negative directions | 220.0
		public static final double TURRET_MINCONTROLANGLE = -35.0; // -25.0
		public static final double TURRET_MAXINITIALANGLE = 220.0; // 220.0
		public static final double TURRET_MININITIALLANGLE = -35.0; //-25.0

		//Turret pose with respect to the robot's center
		public static final double kXOffset = -7.50;//-4.25;
		public static final double kYOffset = 0.0;

		public static final double kEncoderStartingAngle = Settings.kIsUsingCompBot ? 90.0 : 0.0; //40.2 : -110.5; // Absolute position of the magnet 309.25
		public static final double kAngleTolerance = 1.0;

		//Ratios
		public static final double kInternalEncToOutputRatio = ((140.0 * 40.0) / (10.0 * 8.0));//((148.0 * 50.0) / (8.0 * 12.0));//((213 * 50) / (8 * 18));//100.0;
		public static final double kEncoderToOutputRatio = 14.0;//1.0;

		public static final List<double[]> kVisionRanges = Arrays.asList(
			new double[] {TURRET_MINCONTROLANGLE, 87.0},
			new double[] {93.0, TURRET_MAXCONTROLANGLE}
		);
	}

	public static class IntakePivot {

		//IntakePivot Gains
		public static double INTAKEPIVOT_KP = 0.3; // 0.7;
		public static double INTAKEPIVOT_KI = 0.0;
		public static double INTAKEPIVOT_KD = 0.0;// 0.0;
		public static double INTAKEPIVOT_KF = 0.0465;
		public static int INTAKEPIVOT_IZONE = (int) (1023.0 / INTAKEPIVOT_KP);
		public static double INTAKEPIVOT_RAMPRATE = 0;
		public static int INTAKEPIVOT_ALLOWED_ERROR = 100;

		//IntakePivot Constants
		public static final double INTAKEPIVOT_MAXSPEED = 22000.0;//22000.0;

		public static final double INTAKEPIVOT_MAXCONTROLANGLE = 210.0; //In both positive and negative directions | 220.0
		public static final double INTAKEPIVOT_MINCONTROLANGLE = -35.0; // -25.0
		public static final double INTAKEPIVOT_MAXINITIALANGLE = 220.0; // 220.0
		public static final double INTAKEPIVOT_MININITIALLANGLE = -35.0; //-25.0

		//IntakePivot pose with respect to the robot's center
		public static final double kXOffset = -7.50;//-4.25;
		public static final double kYOffset = 0.0;

		public static final double kEncoderStartingAngle = Settings.kIsUsingCompBot ? 0.0 : 0.0; //40.2 : -110.5; // Absolute position of the magnet 309.25
		public static final double kAngleTolerance = 1.0;

		//Ratios
		public static final double kInternalEncToOutputRatio = ((42.0 * 64.0 *  44.0) / (9.0 * 14.0 * 16.0));//((148.0 * 50.0) / (8.0 * 12.0));//((213 * 50) / (8 * 18));//100.0;
		public static final double kEncoderToOutputRatio = 14.0;//1.0;

		public static final List<double[]> kVisionRanges = Arrays.asList(
			new double[] {INTAKEPIVOT_MINCONTROLANGLE, 87.0},
			new double[] {93.0, INTAKEPIVOT_MAXCONTROLANGLE}
		);
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

	//Field Landmarks
	public static final double kFieldLength = 629.25;
	public static final double kFieldHalfWidth = 161.6;

	//Field Positions
	// 135.0 inches from full field to start line pose
	public static final Pose2d testTrenchBallOne = new Pose2d(new Translation2d(kFieldLength - (135.0 + 109.0), 133.0), Rotation2d.fromDegrees(180.0));
	public static final Pose2d testTrenchBallThree = new Pose2d(new Translation2d(kFieldLength - (135.0 + 182.0), 133.0), Rotation2d.fromDegrees(180.0));
	public static final Pose2d testTrenchBallFourAndFive = new Pose2d(new Translation2d(kFieldLength - (135.0 + 247.0), 133.0), Rotation2d.fromDegrees(180.0));
	public static final Pose2d testEnemyTrenchBallFourAndFive = new Pose2d(new Translation2d(kFieldLength - (135.0 + 118.0), -130.0), Rotation2d.fromDegrees(180.0));

	public static final Pose2d testRZoneBall1 = new Pose2d(new Translation2d(kFieldLength - (135.0 + 109.0), (kFieldHalfWidth - 137.0)), Rotation2d.fromDegrees(180.0));
	public static final Pose2d testRZoneBall2 = new Pose2d(new Translation2d(kFieldLength - (135.0 + 126.0), (kFieldHalfWidth - 130.0)), Rotation2d.fromDegrees(180.0));
	public static final Pose2d testRZoneBall3 = new Pose2d(new Translation2d(kFieldLength - (135.0 + 144.0), (kFieldHalfWidth - 127.0)), Rotation2d.fromDegrees(180.0));
	public static final Pose2d testRZoneBall4 = new Pose2d(new Translation2d(kFieldLength - (135.0 + 120.0), -(kFieldHalfWidth - 155.0)), Rotation2d.fromDegrees(180.0));
	public static final Pose2d testRZoneBall5 = new Pose2d(new Translation2d(kFieldLength - (135.0 + 138.0), (kFieldHalfWidth - 159.0)), Rotation2d.fromDegrees(180.0));
		
	public static final Pose2d goalCenterPose = new Pose2d(new Translation2d(kFieldLength - (135.0), (kFieldHalfWidth - 91.0)), Rotation2d.fromDegrees(180.0));
	public static final Pose2d goalWallPose = new Pose2d(new Translation2d(kFieldLength - (135.0), (kFieldHalfWidth - 19.0)), Rotation2d.fromDegrees(180.0));
	public static final Pose2d loadWallPose = new Pose2d(new Translation2d(kFieldLength - (135.0), -(kFieldHalfWidth - 19.0)), Rotation2d.fromDegrees(180.0));
	public static final Pose2d goalWallResetPose = new Pose2d(new Translation2d(495.0, 142.0), Rotation2d.fromDegrees(0.0));
	public static final Pose2d goalCenterOffsetLeftPose = new Pose2d(new Translation2d(kFieldLength - (135.0 + 44.0), (kFieldHalfWidth - 126.0)), Rotation2d.fromDegrees(180.0));
	

	public static final Pose2d goalCenterBackwardPose = new Pose2d(new Translation2d(130.0, -70.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d goalWallBackwardPose = new Pose2d(new Translation2d(130.0, -142.0), Rotation2d.fromDegrees(180.0));
	public static final Pose2d loadWallBackwardPose = new Pose2d(new Translation2d(135.0, 142.0), Rotation2d.fromDegrees(0.0));
	
	public static final Pose2d goalCenterOffsetPose = new Pose2d(new Translation2d(455.0, 80.0), Rotation2d.fromDegrees(0.0));
	public static final Pose2d goalCenterOffsetBackwardPose = new Pose2d(new Translation2d(170.0, -80.0), Rotation2d.fromDegrees(0.0));

	public static final Pose2d goalCenterOffsetLeft180Pose = new Pose2d(new Translation2d(495.0, 25.0), Rotation2d.fromDegrees(180.0));

    // public static final Pose2d goalCenterOffsetLeftPose = new Pose2d(new Translation2d(450.0, 25.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d goalCenterOffsetLeftBackwardPose = new Pose2d(new Translation2d(175.0, -25.0), Rotation2d.fromDegrees(0.0));
	public static final Pose2d goalCenterOffsetResetPose = new Pose2d(new Translation2d(495.0, 25.0), Rotation2d.fromDegrees(180.0));

	public static final Pose2d outsideRZoneLeftPose = new Pose2d(new Translation2d(370.0, -50.0), Rotation2d.fromDegrees(70.0));
    public static final Pose2d rZoneBallOneAnglePose = new Pose2d(new Translation2d(400.0, 20.0), Rotation2d.fromDegrees(70.0));
    public static final Pose2d rZoneInsidePose = new Pose2d(new Translation2d(360.0, 20.0), Rotation2d.fromDegrees(70.0));

    public static final Pose2d trenchBallOnePoseActual = new Pose2d(new Translation2d(385.0, 134.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d trenchBallTwoPoseActual = new Pose2d(new Translation2d(350.0, 134.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d trenchBallThreePoseActual = new Pose2d(new Translation2d(313.0, 134.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d trenchBallFourPoseActual = new Pose2d(new Translation2d(248.0, 134.0), Rotation2d.fromDegrees(0.0));
    
    public static final Pose2d trenchBallOnePose = new Pose2d(new Translation2d(385.0, 128.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d trenchBallTwoPose = new Pose2d(new Translation2d(350.0, 128.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d trenchBallThreePose = new Pose2d(new Translation2d(313.0, 128.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d trenchBallFourPose = new Pose2d(new Translation2d(248.0, 128.0), Rotation2d.fromDegrees(0.0));

    public static final Pose2d trenchBallOneBackwardPose = new Pose2d(new Translation2d(240.0, -128.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d trenchBallTwoBackwardPose = new Pose2d(new Translation2d(275.0, -128.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d trenchBallThreeBackwardPose = new Pose2d(new Translation2d(312.0, -128.0), Rotation2d.fromDegrees(180.0));
    public static final Pose2d trenchBallFourBackwardPose = new Pose2d(new Translation2d(377.0, -128.0), Rotation2d.fromDegrees(0.0));

    public static final Pose2d enemyTrenchPose = new Pose2d(new Translation2d(390.0, -135.0), Rotation2d.fromDegrees(180.0));
    public static final Pose2d enemyTrenchBackwardPose = new Pose2d(new Translation2d(235.0, 135.0), Rotation2d.fromDegrees(0.0));

    public static final Pose2d rZoneBallOnePose = new Pose2d(new Translation2d(387.0, 24.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d rZoneBallTwoPose = new Pose2d(new Translation2d(369.0, 31.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d rZoneBallThreePose = new Pose2d(new Translation2d(350.0, 40.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d rZoneBallFourPose = new Pose2d(new Translation2d(376.0, 6.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d rZoneBallFivePose = new Pose2d(new Translation2d(357.0, 2.0), Rotation2d.fromDegrees(0.0));

    public static final Pose2d rendezvousFourBallCenter = new Pose2d(new Translation2d(362.0, 20.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d rendezvousFourBallOffset = new Pose2d(new Translation2d(370.0, -40.0), Rotation2d.fromDegrees(0.0));

    public static final Pose2d rZoneBallOneBackwardPose = new Pose2d(new Translation2d(235.0, -63.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d rZoneBallTwoBackwardPose = new Pose2d(new Translation2d(233.0, -42.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d rZoneBallThreeBackwardPose = new Pose2d(new Translation2d(226.0, -18.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d rZoneBallFourBackwardPose = new Pose2d(new Translation2d(232.0, 0.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d rZoneBallFiveBackwardPose = new Pose2d(new Translation2d(239.0, 13.0), Rotation2d.fromDegrees(0.0));

    public static final Pose2d rZoneBallOne110Pose = new Pose2d(new Translation2d(390.0, 60.0), Rotation2d.fromDegrees(-110.0));

    public static final Pose2d enemyRZoneBallOnePose = new Pose2d(new Translation2d(237.0, -25.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d enemyRZoneBallTwoPose = new Pose2d(new Translation2d(257.0, -32.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d enemyRZoneBallThreePose = new Pose2d(new Translation2d(275.0, -39.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d enemyRZoneBallFourPose = new Pose2d(new Translation2d(250.0, 5.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d enemyRZoneBallFivePose = new Pose2d(new Translation2d(268.0, -2.0), Rotation2d.fromDegrees(0.0));
}
