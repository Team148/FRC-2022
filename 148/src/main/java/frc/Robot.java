package frc;

import java.util.Arrays;

// import frc.Constants.MotorizedHood;
import frc.auto.AutoModeBase;
import frc.auto.AutoModeExecuter;
import frc.auto.modes.ThiefAndRendezvous;
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
import frc.subsystems.Hanger.HangerState;
import frc.subsystems.Feeder.FeederState;
import frc.subsystems.MotorizedHood;

import com.team1323.io.Xbox;
import com.team1323.lib.util.CrashTracker;
import com.team1323.lib.util.Logger;
import com.team1323.lib.util.Util;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.TrajectoryGenerator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID ;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

	private Looper enabledLooper = new Looper();
	private Looper disabledLooper = new Looper();

	private RobotState robotState = RobotState.getInstance();
	private DriverStation ds = DriverStation.getInstance();

	private Xbox driver, operator;
	private final boolean oneControllerMode = false;
	private boolean flickRotation = false;
	private boolean robotCentric = false;
	private boolean isTurret180Rotation = false;

	private double shooterSpeed = 0.0;

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
		subsystems = new SubsystemManager(
				Arrays.asList(s, swerve, turret, shooter, intake, pivot, hanger, feeder, hood)
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
		swerve.stop();

		autoSelected.initWithDefaults();

		Settings.initializeToggles();

		generator.generateTrajectories();

		AutoModeBase auto = new ThiefAndRendezvous();
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

			hanger.setState(HangerState.OFF);
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

		if (driver.rightCenterClick.longPressed()) {
			swerve.temporarilyDisableHeadingController();
			swerve.zeroSensors(Constants.goalWallResetPose);
			swerve.resetAveragedDirection();
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

		if(driver.rightTrigger.isBeingPressed()) {
			intakePercent = 0.69; //nice
			feeder.setState(FeederState.INTAKING);
		}
		else if(driver.leftTrigger.isBeingPressed()) {
			intakePercent = -1.0;
		}else if(driver.rightTrigger.longReleased() || driver.rightTrigger.shortReleased()){
			feeder.setState(FeederState.OFF);
		}
		
		if (driver.leftBumper.isBeingPressed()){
			pivot.setPosition(0.0);
		} 

		if (driver.rightBumper.isBeingPressed()) {
			pivot.setPosition(5.0);
		}

		if(driver.backButton.isBeingPressed()) {
			hanger.setState(HangerState.EXTEND);
		}
		else if(driver.startButton.isBeingPressed()) {
			hanger.setState(HangerState.STOW);
		}
		else if(driver.backButton.longReleased() || driver.backButton.shortReleased() || driver.startButton.longReleased() || driver.startButton.shortReleased()) {
			hanger.setState(HangerState.OFF);
		}
		
		//OPERATOR

		double operatorRightX = operator.getRightX();//getX(Hand.kRight);

		if (Math.abs(operatorRightX) != 0) {
			turret.setOpenLoop(operatorRightX);
		} else if (turret.isOpenLoop()) {
			turret.lockAngle();
		}


		if(operator.aButton.wasActivated()){
           	shooterSpeed = Constants.Shooter.AT_GOAL;
        }
        else if(operator.bButton.wasActivated()){
			shooterSpeed = Constants.Shooter.BACK_LINE;
        }
        else if(operator.xButton.wasActivated()){ 
			shooterSpeed = Constants.Shooter.HP_WALL;
		}
		else if(operator.yButton.wasActivated()){
			shooterSpeed = Constants.Shooter.LAUNCH_PAD;
		}

		if (operator.POV0.wasActivated()) {
			s.turretPositionState(0.0);
			// System.out.println("Setting turret to 0.0");
		} else if (operator.POV90.wasActivated()) {
			s.turretPositionState(90.0);
			// System.out.println("Setting turret to 90.0");
		} else if (operator.POV180.wasActivated()) {
			s.turretPositionState(180.0);
			// System.out.println("Setting turret to 180.0");
		} else if (operator.POV270.wasActivated()) {
			s.turretPositionState(-20.0);
			// System.out.println("Setting turret to -20.0");
		}

		if(operator.leftTrigger.isBeingPressed()) {
			s.firingVision();
		}

		if(operator.rightTrigger.isBeingPressed() || operator.rightTrigger.longPressed()) {
			feeder.setState(FeederState.SHOOTING);
        }
		else if(operator.rightTrigger.shortReleased() || operator.rightTrigger.longReleased()) {
			feeder.setState(FeederState.OFF);
		}

		if (operator.leftBumper.isBeingPressed() || operator.leftBumper.longPressed()) {
			feeder.setState(FeederState.UNJAM_FEED);
		}
		else if (operator.leftBumper.shortReleased() || operator.leftBumper.longReleased()){
			feeder.setState(FeederState.OFF);
		}

		if (operator.rightCenterClick.shortReleased() || operator.rightCenterClick.longReleased()) {
			if (!turret.isGoingToPole()) {
				if (Math.abs(turret.getRotation().distance(Rotation2d.fromDegrees(180.0))) < Math.abs(turret.getRotation().distance(Rotation2d.fromDegrees(0.0)))) {
					turret.setPosition(180.0);
					isTurret180Rotation = true;
				} else {
					turret.setPosition(0.0);
					isTurret180Rotation = false;
				}
			} else {
				if (isTurret180Rotation) {
					turret.setPosition(0.0);
					isTurret180Rotation = false;
				} else {
					turret.setPosition(180.0);
					isTurret180Rotation = true;
				}
			}
		}

		
		

		intake.setMotor(intakePercent);

		if(shooterSpeed == 0)
		{
			shooter.setOpenLoop(0.0);
		}
		else{
			shooter.setVelocity(shooterSpeed);
		}
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
			swerve.zeroSensors(Constants.goalWallResetPose);
			swerve.resetAveragedDirection();
		}
	}
}