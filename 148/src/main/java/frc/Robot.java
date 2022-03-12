package frc;

import java.util.Arrays;

// import frc.Constants.MotorizedHood;
import frc.auto.AutoModeBase;
import frc.auto.AutoModeExecuter;
import frc.auto.modes.TwoBallAndTerminal;
import frc.auto.SmartDashboardInteractions;
import frc.loops.LimelightProcessor;
import frc.loops.Looper;
import frc.loops.QuinticPathTransmitter;
import frc.loops.RobotStateEstimator;
import frc.subsystems.BallIntake;
import frc.subsystems.IntakePivot;
import frc.subsystems.Shooter;
import frc.subsystems.Hanger;
import frc.subsystems.Feeder;
import frc.subsystems.SubsystemManager;
import frc.subsystems.Superstructure;
import frc.subsystems.Swerve;
import frc.subsystems.Turret;
import frc.subsystems.FalconHood.HoodState;
import frc.subsystems.Hanger.HangerState;
import frc.subsystems.IntakePivot.PivotState;
import frc.subsystems.Feeder.FeederState;
import frc.subsystems.MotorizedHood;
import frc.subsystems.Pigeon;
import frc.subsystems.LinearHood;
import frc.subsystems.FalconHood;
import frc.subsystems.LEDs;

import com.team1323.io.Xbox;
import com.team1323.lib.util.CrashTracker;
import com.team1323.lib.util.Logger;
import com.team1323.lib.util.Util;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.TrajectoryGenerator;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID ;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cscore.UsbCamera;

public class Robot extends TimedRobot {

	private boolean compBot = Constants.kIsUsingCompBot;

	private Superstructure s;
	private SubsystemManager subsystems;
	private Swerve swerve;
	private Turret turret;
	private BallIntake intake;
	private IntakePivot pivot;
	private Hanger hanger;
	private Shooter shooter;
	private static Feeder feeder;
	private MotorizedHood hood;
	private LinearHood newHood;
	private FalconHood falconHood;

	// private LEDs lightShow;
	private LimelightProcessor limelight;

	private AutoModeExecuter autoModeExecuter = null;
	private TrajectoryGenerator generator = TrajectoryGenerator.getInstance();
	private QuinticPathTransmitter qTransmitter = QuinticPathTransmitter.getInstance();
	private SmartDashboardInteractions autoSelected = new SmartDashboardInteractions();

	public static Swerve getSwerve(){
		return Swerve.getInstance();
	}
	public static Turret getTurret(){
		return Turret.getInstance();
	}
	public static MotorizedHood getHood(){
		return MotorizedHood.getInstance();
	}
	public static Shooter getShooter(){
		return Shooter.getInstance();
	}
	public static Feeder getFeeder(){
		return Feeder.getInstance();
	}
	public static Hanger getHanger() {
		return Hanger.getInstance();
	}
	public static IntakePivot getIntakePivot() {
		return IntakePivot.getInstance();
	}
	public static LinearHood getLinearHood() {
		return LinearHood.getInstance();
	}

	public static FalconHood getFalconHood() {
		return FalconHood.getInstance();
	}

	// public static LEDs getLEDs() {
	// 	return LEDs.getInstance();
	// }

	private Looper enabledLooper = new Looper();
	private Looper disabledLooper = new Looper();

	private RobotState robotState = RobotState.getInstance();
	private DriverStation ds = DriverStation.getInstance();

	private Xbox driver, operator;
	private final boolean oneControllerMode = false;
	private boolean flickRotation = false;
	private boolean robotCentric = false;
	private boolean isTurret180Rotation = false;

	private boolean climbModeActivated = false;
	
	private boolean shooter_debug = true;

	private double shooterSpeed = 0.0;
	private double hoodAngle = 0.0;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		s = Superstructure.getInstance();
		swerve = Swerve.getInstance();
		turret = Turret.getInstance();
		shooter = Shooter.getInstance();
		intake = BallIntake.getInstance();
		pivot = IntakePivot.getInstance();
		hanger = Hanger.getInstance();
		feeder = Feeder.getInstance();
		hood = frc.subsystems.MotorizedHood.getInstance();
		newHood = LinearHood.getInstance();
		falconHood = FalconHood.getInstance();
		// lightShow = LEDs.getInstance();
		subsystems = new SubsystemManager(
				Arrays.asList(s, swerve, turret, shooter, intake, pivot, feeder, hood, newHood, falconHood, hanger)
				);

		limelight = LimelightProcessor.getInstance();
		driver = new Xbox(0);
		operator = new Xbox(1);
		driver.setDeadband(0.0);
		operator.setDeadband(0.4);
		operator.rightBumper.setLongPressDuration(1.0);

		Logger.clearLog();

		enabledLooper.register(RobotStateEstimator.getInstance());
		enabledLooper.register(QuinticPathTransmitter.getInstance());
		enabledLooper.register(LimelightProcessor.getInstance());
		disabledLooper.register(RobotStateEstimator.getInstance());
		disabledLooper.register(QuinticPathTransmitter.getInstance());
		disabledLooper.register(LimelightProcessor.getInstance());
		subsystems.registerEnabledLoops(enabledLooper);
		subsystems.registerDisabledLoops(disabledLooper);

		swerve.zeroSensors();
		turret.zeroSensors();
		falconHood.zeroSensors();
		pivot.zeroSensors();
		swerve.stop();

		autoSelected.initWithDefaults();

		Settings.initializeToggles();

		UsbCamera camera = CameraServer.startAutomaticCapture();
		camera.setFPS(10);
		camera.setResolution(320, 240);

		generator.generateTrajectories();

		AutoModeBase auto = new TwoBallAndTerminal();
		qTransmitter.addPaths(auto.getPaths());
		System.out.println("Total path time: " + qTransmitter.getTotalPathTime(auto.getPaths()));
	}

	public void allPeriodic() {
		subsystems.outputToSmartDashboard();
		robotState.outputToSmartDashboard();
		enabledLooper.outputToSmartDashboard();
		SmartDashboard.putBoolean("Enabled", ds.isEnabled());
		SmartDashboard.putNumber("Match time", ds.getMatchTime());
	}

	public void autoConfig() {
		swerve.zeroSensors();
		swerve.setNominalDriveOutput(0.0);
		swerve.requireModuleConfiguration();
	}

	public void teleopConfig() {
		swerve.setNominalDriveOutput(0.0);
		swerve.set10VoltRotationMode(false);
	}

	@Override
	public void autonomousInit() {
		try {
			if (autoModeExecuter != null)
				autoModeExecuter.stop();

			autoConfig();

			disabledLooper.stop();
			enabledLooper.start();

			s.turret.zeroedTurret();

			SmartDashboard.putBoolean("Auto", true);

			autoModeExecuter = new AutoModeExecuter();
			autoModeExecuter.setAutoMode(autoSelected.getSelectedAutoMode());
			autoModeExecuter.start();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		allPeriodic();
	}

	@Override
	public void teleopInit() {
		try {
			disabledLooper.stop();
			enabledLooper.start();

			s.turret.zeroedTurret();
			limelight.ledOn(true);

			teleopConfig();
			SmartDashboard.putBoolean("Auto", false);

			// hanger.setState(HangerState.OFF);

			// lightShow.conformToState(LEDs.State.BREATHING_RED);
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {

		try {
			driver.update();
			operator.update();

			if (oneControllerMode)
				oneControllerMode();
			else
				twoControllerMode();

			allPeriodic();

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void disabledInit() {
		try {
			if (autoModeExecuter != null)
				autoModeExecuter.stop();
				enabledLooper.stop();
			subsystems.stop();
			disabledLooper.start();

			limelight.ledOn(false);
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void disabledPeriodic() {
		try {
			turret.resetToAbsolute();
			// lightShow.conformToState(LEDs.State.RED);
			Settings.update();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void testInit() {

	}

	@Override
	public void testPeriodic() {
		allPeriodic();
	}

	private void twoControllerMode() {
		double intakePercent = 0.0;

		//DRIVE CONTROLS
		// if (operator.backButton.isBeingPressed()) {
		// 	s.neutralState();
		// }

		double swerveYInput = -driver.getLeftX();//getX(Hand.kLeft);
		double swerveXInput = driver.getLeftY();//getY(Hand.kLeft);
		double swerveRotationInput = (flickRotation ? 0.0 : driver.getRightX());//getX(Hand.kRight));

		swerve.sendInput(swerveXInput, swerveYInput, swerveRotationInput, robotCentric, driver.leftBumper.isBeingPressed());

		if (flickRotation) {
			swerve.updateControllerDirection(new Translation2d(-driver.getRightY(), driver.getRightX()));//getY(Hand.kRight), driver.getX(Hand.kRight)));
			if (!Util.epsilonEquals(
					Util.placeInAppropriate0To360Scope(swerve.getTargetHeading(),
							swerve.averagedDirection.getDegrees()),
					swerve.getTargetHeading(), swerve.rotationDivision / 2.0)) {
				swerve.rotate(swerve.averagedDirection.getDegrees());
			}
		}

		if (driver.backButton.shortReleased() || driver.backButton.longPressed()) {
			swerve.temporarilyDisableHeadingController();
			swerve.zeroSensors(Constants.swerveReset);
			swerve.resetAveragedDirection();
		}

		if (driver.startButton.longPressed()) {
			swerve.lazyReset();
			s.turretPositionState(90.0);
			hoodAngle = 0.0;
			shooterSpeed = 0.0;
			pivot.setState(PivotState.RESET);
		}

		//STATE CONTROLS
		// double operaterLeftStick = operator.getY(Hand.kLeft);
		// double operatorRightStick = operator.getY(Hand.kRight);

		//DRIVER

		if (driver.bButton.isBeingPressed()){
			swerve.rotate(90);
		}
		else if (driver.aButton.isBeingPressed()){
			swerve.rotate(180);
		}
		else if (driver.xButton.isBeingPressed()){
			swerve.rotate(270);
		}
		else if (driver.yButton.isBeingPressed()){
			swerve.rotate(0);
		}

		if(driver.leftBumper.wasActivated()) {
			pivot.setState(PivotState.UP);
		}
		else if(driver.rightBumper.wasActivated()) {
			pivot.setState(PivotState.DOWN);
		}

		if(driver.rightTrigger.isBeingPressed()) {
			intakePercent = -1.0;
			feeder.setState(FeederState.INTAKING);
			// lightShow.conformToState(LEDs.State.RAPID_FLASHING_YELLOW);//quiero más Ball
		}
		else if(driver.leftTrigger.isBeingPressed()) {
			intakePercent = 1.0;
		}else if(driver.rightTrigger.longReleased() || driver.rightTrigger.shortReleased()){
			feeder.setState(FeederState.OFF);
		}

		//OPERATOR

		double operatorRightX = operator.getRightX();//getX(Hand.kRight);
		double operatorLeftY = operator.getLeftY();

		//optional manual shooter controls
				// + (X) -

		if(shooter_debug){
			if(operator.backButton.wasActivated()) {
				hoodAngle += 1.0;
				System.out.println("Hood Angle Set to: " + hoodAngle + " degrees.");
			}
			else if(operator.startButton.wasActivated()) {
				hoodAngle -= 1.0;
				System.out.println("Hood Angle Set to: " + hoodAngle + " degrees.");
			}

			if(operator.leftBumper.wasActivated()) {
				shooterSpeed += 250.0;
				System.out.println("Shooter Speed Set to: " + shooterSpeed);
			}
			else if(operator.rightBumper.wasActivated()) {
				shooterSpeed -= 250.0;
				System.out.println("Shooter Speed Set to: " + shooterSpeed);
			}
		}// end of shooter testing

		if (Math.abs(operatorRightX) != 0) {
			turret.setOpenLoop(operatorRightX);
		} else if (turret.isOpenLoop()) {
			turret.lockAngle();
		}

		// if(operator.backButton.isBeingPressed()) {
		// 	hanger.setMotor(1.0);
		// }
		// else if (operator.startButton.isBeingPressed()) {
		// 	hanger.setMotor(-1.0);
		// }
		// else if (operator.backButton.longReleased() || operator.backButton.shortReleased() || operator.startButton.longReleased() || operator.startButton.shortReleased()) {
		// 	hanger.setMotor(0.0);
		// }

		if(operator.backButton.isBeingPressed() && operator.startButton.isBeingPressed()) {
			climbModeActivated = true;
			pivot.setState(PivotState.RESET);
		}

		if(climbModeActivated) {
			if(operator.aButton.isBeingPressed() || operator.aButton.longPressed()){
				hanger.setMotor(0.90);
			}
			else if(operator.bButton.isBeingPressed() || operator.bButton.longPressed()) {
				hanger.setMotor(-0.90);
			}
			else if(operator.xButton.isBeingPressed() || operator.xButton.longPressed()) {
				hanger.setOneClawServo(0.0);
			}
			else if(operator.yButton.isBeingPressed() || operator.yButton.longPressed()) {
				hanger.setTwoClawServo(0.0);
			}
			else if(operator.aButton.shortReleased() || operator.aButton.longReleased() || operator.bButton.shortReleased() || operator.bButton.longReleased()){
				hanger.setMotor(0.0);
			}
			else if(operator.xButton.shortReleased() || operator.xButton.longReleased()){
				hanger.setOneClawServo(0.18);
			}
			else if(operator.yButton.shortReleased() || operator.yButton.longReleased()){
				hanger.setTwoClawServo(0.18);
			}
		}
		else {
			if(operator.aButton.isBeingPressed()){
				hoodAngle = 7.0;
				shooterSpeed = Constants.Shooter.AT_GOAL; 
			}
			else if(operator.bButton.isBeingPressed()){
				shooterSpeed = Constants.Shooter.BACK_LINE;
				hoodAngle = 28.0; //30.0; 
				// falconHood.setHoodPosition(23.0);
			}
			else if(operator.xButton.isBeingPressed()){ 
				shooterSpeed = Constants.Shooter.HP_WALL;
				hoodAngle = 46.0;
				// // falconHood.setHoodPosition(0.0);
				// falconHood.setHoodPosition(48.0);
			}
			else if(operator.yButton.isBeingPressed()){
				shooterSpeed = Constants.Shooter.LAUNCH_PAD + 250;
				hoodAngle = 41.0; //42.0;
				// falconHood.setHoodPosition(27.0);
			}
			else if (operator.leftCenterClick.wasActivated()) {
				hoodAngle = 0.0;
				// falconHood.setHoodPosition(0.0);
				shooterSpeed = 0.0;
			}
		}

		if (operator.POV0.wasActivated()) {
			s.turretPositionState(-180.0); //270.0//90
			// System.out.println("Setting turret to 0.0");
		} else if (operator.POV90.wasActivated()) {
			s.turretPositionState(-90.0);//180 //180
			// System.out.println("Setting turret to 90.0");
		} else if (operator.POV180.wasActivated()) {
			s.turretPositionState(0.0);//90 //305
			// System.out.println("Setting turret to 180.0");
		} else if (operator.POV270.wasActivated()) {
			s.turretPositionState(55.0);//305 //35
			// System.out.println("Setting turret to -20.0");
		}

		if(operator.leftTrigger.isBeingPressed()) {
			s.firingVision();
		}

		if(operator.rightTrigger.isBeingPressed() || operator.rightTrigger.longPressed()) {
			intakePercent = -1.0;
			feeder.setState(FeederState.SHOOTING);
			
        }
		if(operator.rightTrigger.shortReleased() || operator.rightTrigger.longReleased()) {
			feeder.setState(FeederState.OFF);
		}
		if(!shooter_debug){
			if(operator.leftBumper.isBeingPressed()) {
				feeder.setState(FeederState.UNJAM_FEED);
			}
			else if(operator.leftBumper.shortReleased() || operator.leftBumper.longReleased()) {
				feeder.setState(FeederState.OFF);
			}
		}

		if (operator.rightCenterClick.shortReleased() || operator.rightCenterClick.longReleased()) {
			if (!turret.isGoingToPole()) {
				if (Math.abs(turret.getRotation().distance(Rotation2d.fromDegrees(0.0))) < Math.abs(turret.getRotation().distance(Rotation2d.fromDegrees(-180.0)))) {
					turret.setPosition(0.0);
					isTurret180Rotation = true;
				} else {
					turret.setPosition(-180.0);
					isTurret180Rotation = false;
				}
			} else {
				if (isTurret180Rotation) {
					turret.setPosition(-180.0);
					isTurret180Rotation = false;
				} else {
					turret.setPosition(0.0);
					isTurret180Rotation = true;
				}
			}
		}

		// shooter.setVelocity(shooterSpeed);
		// falconHood.setHoodPosition(hoodAngle);
		intake.setMotor(intakePercent);
	}

	private void oneControllerMode() {

		double swerveYInput = driver.getLeftX();//getX(Hand.kLeft);
		double swerveXInput = -driver.getLeftY();//getY(Hand.kLeft);
		double swerveRotationInput = (flickRotation ? 0.0 : driver.getRightX());//getX(Hand.kRight));

		swerve.sendInput(swerveXInput, swerveYInput, swerveRotationInput, robotCentric, driver.leftTrigger.isBeingPressed());

		if (flickRotation) {
			swerve.updateControllerDirection(new Translation2d(-driver.getRightY(), driver.getRightX()));//getY(Hand.kRight), driver.getX(Hand.kRight)));
			if (!Util.epsilonEquals(
					Util.placeInAppropriate0To360Scope(swerve.getTargetHeading(),
							swerve.averagedDirection.getDegrees()),
					swerve.getTargetHeading(), swerve.rotationDivision / 2.0)) {
				swerve.rotate(swerve.averagedDirection.getDegrees());
			}
		}

		if (driver.backButton.shortReleased() || driver.backButton.longPressed()) {
			swerve.temporarilyDisableHeadingController();
			swerve.zeroSensors(Constants.postTerminalShot);
			swerve.resetAveragedDirection();
		}
	}
}