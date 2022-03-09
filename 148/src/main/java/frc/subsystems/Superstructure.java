package frc.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import frc.Constants;
import frc.RobotMap;
import frc.RobotState;
import frc.loops.ILooper;
import frc.loops.LimelightProcessor;
import frc.loops.Loop;
import frc.subsystems.requests.LambdaRequest;
import frc.subsystems.requests.ParallelRequest;
import frc.subsystems.requests.Request;
import frc.subsystems.requests.SequentialRequest;
import frc.subsystems.requests.RequestList;
import frc.vision.ShooterAimingParameters;
import com.team1323.lib.util.InterpolatingDouble;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

import frc.subsystems.BallIntake;
import frc.subsystems.Shooter;
import frc.subsystems.Turret;
import frc.subsystems.FalconHood;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Timer;

public class Superstructure extends Subsystem {
	
	public Swerve swerve;
	public Turret turret;
	public Shooter shooter;
	public FalconHood falconhood;

	private RobotState robotState;
	
	public Superstructure(){
		swerve = Swerve.getInstance();
		turret = Turret.getInstance();
		shooter = Shooter.getInstance();
		falconhood = FalconHood.getInstance();

		robotState = RobotState.getInstance();
		
		queuedRequests = new ArrayList<>(0);
	}
	private static Superstructure instance = null;
	public static Superstructure getInstance(){
		if(instance == null)
			instance = new Superstructure();
		return instance;
	}
	
	private Request activeRequest = null;
	private List<Request> queuedRequests = new ArrayList<>();
	
	private boolean newRequest = false;
	private boolean activeRequestsCompleted = false;
	private boolean allRequestsCompleted = false;
	public boolean requestsCompleted(){ return allRequestsCompleted; }
	
	private void setActiveRequest(Request request){
		activeRequest = request;
		newRequest = true;
		allRequestsCompleted = false;
	}
	
	private void setQueue(List<Request> requests){
		clearQueue();
		for(Request request : requests) {
			queuedRequests.add(request);
		}
	}

	private void setQueue(Request request) {
		setQueue(Arrays.asList(request));
	}

	private void clearQueue() {
		queuedRequests.clear();
	}
	
	public void request(Request r){
		setActiveRequest(r);
		clearQueue();
	}
	
	public void request(Request active, Request queue){
		setActiveRequest(active);
		setQueue(queue);
	}
	
	public void queue(Request request){
		queuedRequests.add(request);
	}
	
	public void replaceQueue(Request request){
		setQueue(request);
	}
	
	boolean isPrefire = false;
	private final Loop loop = new Loop(){

		@Override
		public void onStart(double timestamp) {
			stop();
		}

		@Override
		public void onLoop(double timestamp) {
			synchronized(Superstructure.this){
				Optional<ShooterAimingParameters> aim = robotState.getAimingParameters(false);

				// if (swerve.modules.get(0).getModuleVelocity() >= 60.0 && intake.getState() == Intake.State.INTAKE) {
				// 	intake.setOpenLoop(Constants.Intake.kFastIntakeSpeed);
				// } else if (swerve.modules.get(0).getModuleVelocity() < 60.0 && intake.getState() == Intake.State.INTAKE) {
				// 	intake.setOpenLoop(Constants.Intake.kIntakeSpeed);
				// }

				if (turret.isTracking()) {
					if (aim.isPresent()) {
						if ((!turret.inTurretRange(aim.get().getTurretAngle().getDegrees()) || 
							!turret.inVisionRange(turret.boundToTurretRange(aim.get().getTurretAngle().getDegrees())))
							&& !swerve.isGoingToPole()) {
							//swerve.rotate(swerve.closestPole());
						}
						double temp_range = aim.get().getRange();
						falconhood.setHoodPosition(Constants.kVisionAngleTreemap.getInterpolated(new InterpolatingDouble(temp_range)).value);
						shooter.setVelocity(Constants.kVisionSpeedTreemap.getInterpolated(new InterpolatingDouble(temp_range)).value);
					}
				}

				// if (aim.isPresent() && isPrefire) {
				// 	motorizedHood.visionExtension();
				// }

				if(newRequest && activeRequest != null) {
					activeRequest.act();
					newRequest = false;
				} 

				if(activeRequest == null) {
					if(queuedRequests.isEmpty()) {
						allRequestsCompleted = true;
					} else {
						setActiveRequest(queuedRequests.remove(0));
					}
				} else if(activeRequest.isFinished()) {
					activeRequest = null;
				}
			
			}
		}

		@Override
		public void onStop(double timestamp) {
		}
	};

	@Override
	public void stop() {
	}

	@Override
	public void zeroSensors() {
		
	}

	@Override
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(loop);
	}

	@Override
	public void outputTelemetry() {
	}

	public Request waitRequest(double seconds){
		return new Request(){
			double startTime = 0.0;
			double waitTime = 1.0;
		
			@Override
			public void act() {
				startTime = Timer.getFPGATimestamp();
				waitTime = seconds;
			}

			@Override
			public boolean isFinished(){
				return (Timer.getFPGATimestamp() - startTime) > waitTime;
			}
		};
	}

	public Request waitForVisionRequest(){
		return new Request(){

			@Override
			public void act() {

			}

			@Override
			public boolean isFinished(){
				return robotState.seesTarget();
			}

		};
	}


	private boolean needsToNotifyDrivers = false;
    public boolean needsToNotifyDrivers() {
        if (needsToNotifyDrivers) {
            needsToNotifyDrivers = false;
            return true;
        }
        return false;
    }
	public SequentialRequest postShootingRequest() {
		return new SequentialRequest (
			new ParallelRequest(
				new LambdaRequest(() -> LimelightProcessor.getInstance().ledOn(false)),
				new LambdaRequest(() -> needsToNotifyDrivers = true),
				new LambdaRequest(() -> isPrefire = false),
				turret.stateRequest(Turret.State.POSITION)
			)
		);
	}

	public ParallelRequest autoPostShootingRequest() {
		return new ParallelRequest(
			new LambdaRequest(() -> LimelightProcessor.getInstance().ledOn(false)),
			turret.angleRequest(180.0)
		);
	}

	/////States/////

	public void neutralState() {
		request(new ParallelRequest(
			// feeder.stateRequest(Feeder.State.OFF),
			// intake.stateRequest(Intake.State.OFF),
			// shooter.openLoopRequest(0.0, 0.0),
			new LambdaRequest(() -> LimelightProcessor.getInstance().ledOn(true)),
			new LambdaRequest(() -> isPrefire = false),
			// motorizedHood.angleRequest(Constants.MotorizedHood.kMinControlAngle),
			turret.stateRequest(Turret.State.POSITION)
		));
	}

	// public void autoNeutralState() {
	// 	request(
	// 		new ParallelRequest(
	// 			//feeder.stateRequest(Feeder.State.OFF),
	// 			intake.stateRequest(Intake.State.OFF)				
	// 		)
	// 	);
	// }

	public void autoPostShootingState() {
		request(
			new ParallelRequest(
				autoPostShootingRequest()				
			)
		);
	}

	// public void intakeState() {
	// 	request(new ParallelRequest(
	// 		intake.stateRequest(Intake.State.INTAKE),
	// 		feeder.stateRequest(Feeder.State.RECEIVING),
	// 		shooter.openLoopRequest(0.0, 0.0)
	// 	));
	// }

	// public void autoIntakeState() {
	// 	request(new ParallelRequest(
	// 		intake.stateRequest(Intake.State.INTAKE),
	// 		feeder.stateRequest(Feeder.State.RECEIVING)
	// 	));
	// }

	// public void postShooting() {
	// 	request(
	// 		new SequentialRequest(
	// 			feeder.waitToFinishShootingRequest(),
	// 			postShootingRequest(),
	// 			feeder.stateRequest(Feeder.State.OFF)
	// 		)
	// 	);
	// }

	public void postManualShooting() {
		request(
			new SequentialRequest(
				postShootingRequest()
			)
		);
	}

	/**
	 * Moves the robot into the specified distance from the target
	 * @param translationFromTarget Negative distance means in front of the target and positive means behind the target
	 */
	public void moveIntoRangeState(Translation2d translationFromTarget) {
		request(
			new SequentialRequest(
				new LambdaRequest(() -> LimelightProcessor.getInstance().ledOn(true)),
				swerve.visionPIDRequest(translationFromTarget)
			)
		);
	}

	/**
	 * Moves the robot into the specified distance from the target and with final orientation
	 * @param translationFromTarget Negative distance means in front of the target and positive means behind the target
	 * @param approachAngle The angle the robot can be away from the target.
	 */
	public void moveIntoRangeState(Translation2d translationFromTarget, Rotation2d approachAngle) {
		request(
			new SequentialRequest(
				new LambdaRequest(() -> LimelightProcessor.getInstance().ledOn(true)),
				swerve.visionPIDRequest(translationFromTarget, approachAngle)
			)
		);
	}

	// public void firingState() {
	// 	request(
	// 		new SequentialRequest(
	// 			new ParallelRequest(
	// 				//hood.stateRequest(Hood.State.CLOSE_FIRE),
	// 				//actuatingHood.stateRequest(ActuatingHood.State.MID),
	// 				shooter.visionHoldWhenReadyRequest()
	// 			),
	// 			new ParallelRequest(
	// 				feeder.stateRequest(Feeder.State.FEEDING),
	// 				intake.stateRequest(Intake.State.FEEDING_INTAKE),
	// 				feeder.waitToFinishShootingRequest()
	// 			),
	// 			postShootingRequest()
	// 		)
	// 	);
	// }

	public void moveAndFireVisionState(Translation2d translationFromTarget) {
		request(
			new SequentialRequest(
				new ParallelRequest(
					swerve.visionPIDRequest(translationFromTarget),
					turret.startVisionRequest(false)
					// shooter.visionHoldWhenReadyRequest()
				)
				// new ParallelRequest(
				// 	intake.stateRequest(Intake.State.FEEDING_INTAKE),
				// 	feeder.stateRequest(Feeder.State.FEEDING),
				// 	feeder.waitToFinishShootingRequest()
				// ),
				// postShootingRequest()
			)
		);
	}

	public void moveAndFireVisionState(Translation2d translationFromTarget, Rotation2d approachAngle) {
		request(
			new SequentialRequest(
				new ParallelRequest(
					swerve.visionPIDRequest(translationFromTarget, approachAngle),
					turret.startVisionRequest(false)
					// shooter.visionHoldWhenReadyRequest()
				)
				// new ParallelRequest(
				// 	intake.stateRequest(Intake.State.FEEDING_INTAKE),
				// 	feeder.stateRequest(Feeder.State.FEEDING),
				// 	feeder.waitToFinishShootingRequest()
				// ),
				// postShootingRequest()
			)
		);
	}

	public void closeProtectedMoveAndFireState(Rotation2d approachAngle) {
		request(
			new SequentialRequest(
				new LambdaRequest(() -> LimelightProcessor.getInstance().ledOn(true)),
				new ParallelRequest(
					//actuatingHood.stateRequest(ActuatingHood.State.CLOSE),
					// intake.stateRequest(Intake.State.INTAKE),
					// motorizedHood.angleRequest(Constants.MotorizedHood.kCloseAngle),
					// shooter.holdWhenReadyRequest(Constants.Shooter.kCloseTopRPM, Constants.Shooter.kCloseBottomRPM), // 400.0 3000.0 | 1100 3600
					swerve.visionPIDRequest(new Translation2d(-52.0, 0.0), approachAngle),
					turret.startVisionRequest(false)
				),
				swerve.lockDrivePositionRequest()
				//actuatingHood.waitForHoodAngleRequst(),
				// new ParallelRequest(
				// 	intake.stateRequest(Intake.State.OFF),
				// 	feeder.stateRequest(Feeder.State.FEEDING),
				// 	feeder.waitToFinishShootingRequest()
				// ),
				// postShootingRequest()
			)
		);
	}

	public void closeProtectedMoveAndFireState() {
		request(
			new SequentialRequest(
				new LambdaRequest(() -> LimelightProcessor.getInstance().ledOn(true)),
				new LambdaRequest(() -> isPrefire = false),
				new ParallelRequest(
					// intake.stateRequest(Intake.State.INTAKE),
					// motorizedHood.angleRequest(Constants.MotorizedHood.kCloseAngle),
					// shooter.holdWhenReadyRequest(Constants.Shooter.kCloseTopRPM, Constants.Shooter.kCloseBottomRPM),
					swerve.visionPIDRequest(new Translation2d(-53.0, 0.0)),
					turret.startVisionRequest(false)
				),
				swerve.lockDrivePositionRequest()
				// new ParallelRequest(
				// 	intake.stateRequest(Intake.State.OFF),
				// 	feeder.stateRequest(Feeder.State.FEEDING),
				// 	feeder.waitToFinishShootingRequest()
				// ),
				// postShootingRequest()
			)
		);
	}

	public void midVisionMoveAndFireState() {
		request(
			new SequentialRequest(
				new LambdaRequest(() -> LimelightProcessor.getInstance().ledOn(true)),
				new LambdaRequest(() -> isPrefire = false),
				new ParallelRequest(
					// intake.stateRequest(Intake.State.INTAKE),
					// motorizedHood.angleRequest(Constants.MotorizedHood.kMidAngle),
					// shooter.holdWhenReadyRequest(Constants.Shooter.kMidTopRPM, Constants.Shooter.kMidBottomRPM),
					swerve.visionPIDRequest(new Translation2d(-141.0, 0.0)),
					turret.startVisionRequest(false)
				),
				swerve.lockDrivePositionRequest()
				// new ParallelRequest(
				// 	intake.stateRequest(Intake.State.OFF),
				// 	feeder.stateRequest(Feeder.State.FEEDING),
				// 	feeder.waitToFinishShootingRequest()
				// ),
				// postShootingRequest()
			)
		);
	}

	public void farVisionMoveAndFireState() {
		request(
			new SequentialRequest(
				new LambdaRequest(() -> LimelightProcessor.getInstance().ledOn(true)),
				new LambdaRequest(() -> isPrefire = false),
				new ParallelRequest(
					// intake.stateRequest(Intake.State.INTAKE),
					// motorizedHood.angleRequest(Constants.MotorizedHood.kFarAngle),
					// shooter.holdWhenReadyRequest(Constants.Shooter.kFarTopRPM, Constants.Shooter.kFarBottomRPM),
					swerve.visionPIDRequest(new Translation2d(-235.0, 0.0)),
					turret.startVisionRequest(false)
				),
				swerve.lockDrivePositionRequest()
				// new ParallelRequest(
				// 	intake.stateRequest(Intake.State.OFF),
				// 	feeder.stateRequest(Feeder.State.FEEDING),
				// 	feeder.waitToFinishShootingRequest()
				// ),
				// postShootingRequest()
			)
		);
	}

	// public void deepFireState() {
	// 	request(
	// 		new SequentialRequest(
	// 			new ParallelRequest(
	// 				//hood.stateRequest(Hood.State.FAR_FIRE),
	// 				//actuatingHood.stateRequest(ActuatingHood.State.FAR),
	// 				//motorizedHood.angleRequest(Constants.MotorizedHood.kMaxControlAngle),
	// 				shooter.holdWhenReadyRequest(1400.0, 4400.0)
	// 			),
	// 			//actuatingHood.waitForHoodAngleRequst(),
	// 			new ParallelRequest(
	// 				intake.stateRequest(Intake.State.FEEDING_INTAKE),
	// 				feeder.stateRequest(Feeder.State.FEEDING),
	// 				feeder.waitToFinishShootingRequest()
	// 			),
	// 			postShootingRequest()
	// 		)
	// 	);
	// }


	public void firingVisionState(double topRPM, double bottomRPM) {
		request( 
			new ParallelRequest(
				turret.startVisionRequest(true)
				// new SequentialRequest(
				// 	shooter.holdWhenReadyRequest(topRPM, bottomRPM),
				// 	new ParallelRequest(
				// 		intake.stateRequest(Intake.State.FEEDING_INTAKE),
				// 		feeder.stateRequest(Feeder.State.FEEDING),
				// 		feeder.waitToFinishShootingRequest()
				// 	),
				// 	postShootingRequest()
				// )
			)
		);
	}

	public void firingVision() {
		request( 
			new ParallelRequest(
				turret.startVisionRequest(true)
				// new SequentialRequest(
				// 	shooter.holdWhenReadyRequest(topRPM, bottomRPM),
				// 	new ParallelRequest(
				// 		intake.stateRequest(Intake.State.FEEDING_INTAKE),
				// 		feeder.stateRequest(Feeder.State.FEEDING),
				// 		feeder.waitToFinishShootingRequest()
				// 	),
				// 	postShootingRequest()
				// )
			)
		);
	}
	public void firingVisionOuter() {
		request( 
			new ParallelRequest(
				turret.startVisionRequest(false)
				// new SequentialRequest(
				// 	shooter.holdWhenReadyRequest(topRPM, bottomRPM),
				// 	new ParallelRequest(
				// 		intake.stateRequest(Intake.State.FEEDING_INTAKE),
				// 		feeder.stateRequest(Feeder.State.FEEDING),
				// 		feeder.waitToFinishShootingRequest()
				// 	),
				// 	postShootingRequest()
				// )
			)
		);
	}

	public void turretPositionState(double angle) {
		request(
			new SequentialRequest(
				//hood.stateRequest(Hood.State.CLOSE_FIRE),
				//actuatingHood.stateRequest(ActuatingHood.State.MID),
				turret.angleRequest(angle)
			)
		);
		System.out.println("Requesting turret GOTO angle");
	}

	public void closeShot() {
		request( 
			new SequentialRequest(
				new LambdaRequest(() -> LimelightProcessor.getInstance().ledOn(true)),
				new LambdaRequest(() -> isPrefire = false),
				new ParallelRequest(
					// motorizedHood.angleRequest(Constants.MotorizedHood.kCloseAngle),
					// shooter.holdWhenReadyRequest(Constants.Shooter.kCloseTopRPM, Constants.Shooter.kCloseBottomRPM), // 400.0 3000.0 | 1100 3600
					turret.startVisionRequest(false)
				),
				swerve.lockDrivePositionRequest()
				// new ParallelRequest(
				// 	feeder.stateRequest(Feeder.State.FEEDING),
				// 	feeder.waitToFinishShootingRequest()
				// ),
				// postShootingRequest()
			)
		);
	}

	public void testMidShot() {
		request( 
			new SequentialRequest(
				new LambdaRequest(() -> LimelightProcessor.getInstance().ledOn(true)),
				new ParallelRequest(
					// motorizedHood.angleRequest(Constants.MotorizedHood.kMidAngle),
					// shooter.holdWhenReadyRequest(Constants.Shooter.kMidTopRPM, Constants.Shooter.kMidBottomRPM), // 400.0 3000.0 | 1100 3600
					turret.startVisionRequest(false)
				)
				//swerve.lockDrivePositionRequest(),
				//actuatingHood.waitForHoodAngleRequst(),
				// new ParallelRequest(
				// 	//intake.stateRequest(Intake.State.FEEDING_INTAKE),
				// 	feeder.stateRequest(Feeder.State.FEEDING),
				// 	feeder.waitToFinishShootingRequest()
				// ),
				// postShootingRequest()
			)
		);
	}

	public void midShot() {
		request( 
			new SequentialRequest(
				new LambdaRequest(() -> LimelightProcessor.getInstance().ledOn(true)),
				new LambdaRequest(() -> isPrefire = false),
				new ParallelRequest(
					// motorizedHood.angleRequest(Constants.MotorizedHood.kMidAngle),
					// shooter.holdWhenReadyRequest(Constants.Shooter.kMidTopRPM, Constants.Shooter.kMidBottomRPM), // 400.0 3000.0 | 1100 3600
					turret.startVisionRequest(false)
				),
				swerve.lockDrivePositionRequest()
				// new ParallelRequest(
				// 	feeder.stateRequest(Feeder.State.FEEDING),
				// 	feeder.waitToFinishShootingRequest()
				// ),
				// postShootingRequest()
			)
		);
	}

	public void testFarShot() {
		request( 
			new SequentialRequest(
				new LambdaRequest(() -> LimelightProcessor.getInstance().ledOn(true)),
				new ParallelRequest(
					// motorizedHood.angleRequest(Constants.MotorizedHood.kFarAngle),
					// shooter.holdWhenReadyRequest(Constants.Shooter.kFarTopRPM, Constants.Shooter.kFarBottomRPM), // 400.0 3000.0 | 1100 3300
					turret.startVisionRequest(false)
				)
				// new ParallelRequest(
				// 	feeder.stateRequest(Feeder.State.FEEDING),
				// 	feeder.waitToFinishShootingRequest()
				// ),
				// postShootingRequest()
			)
		);
	}

	public void farShot() {
		request( 
			new SequentialRequest(
				new LambdaRequest(() -> LimelightProcessor.getInstance().ledOn(true)),
				new LambdaRequest(() -> isPrefire = false),
				new ParallelRequest(
					// motorizedHood.angleRequest(Constants.MotorizedHood.kFarAngle),
					// shooter.holdWhenReadyRequest(Constants.Shooter.kFarTopRPM, Constants.Shooter.kFarBottomRPM), // 400.0 3000.0 | 1100 3300
					turret.startVisionRequest(false)
				),
				swerve.lockDrivePositionRequest()
				// new ParallelRequest(
				// 	feeder.stateRequest(Feeder.State.FEEDING),
				// 	feeder.waitToFinishShootingRequest()
				// ),
				// postShootingRequest()
			)
		);
	}

	public void manualCloseShot() {
		request( 
			new SequentialRequest(
				new ParallelRequest(
					// motorizedHood.angleRequest(Constants.MotorizedHood.kCloseAngle),
					// shooter.holdWhenReadyRequest(Constants.Shooter.kCloseTopRPM, Constants.Shooter.kCloseBottomRPM),
					turret.angleRequest(0.0)
				)
				// new ParallelRequest(
				// 	feeder.stateRequest(Feeder.State.FEEDING),
				// 	feeder.waitToFinishShootingRequest()
				// ),
				// postShootingRequest()
			)
		);
	}
	
	public void firingVisionState() {
		request( 
			new SequentialRequest(
				new ParallelRequest(
					//actuatingHood.waitForHoodAngleRequst(),
					//hood.stateRequest(Hood.State.CLOSE_FIRE),
					//actuatingHood.stateRequest(ActuatingHood.State.MID),
					//shooter.visionHoldWhenReadyRequest()
					// shooter.holdWhenReadyRequest(1300.0, 3600.0), // 400.0 3000.0 | 1100 3600
					turret.startVisionRequest(false)
				),
				swerve.lockDrivePositionRequest()
				//actuatingHood.waitForHoodAngleRequst(),
				// new ParallelRequest(
				// 	intake.stateRequest(Intake.State.FEEDING_INTAKE),
				// 	feeder.stateRequest(Feeder.State.FEEDING),
				// 	feeder.waitToFinishShootingRequest()
				// ),
				// postShootingRequest()
			)
		);
	}

	// public void preFireState() {
	// 	request(
	// 		new ParallelRequest(
	// 			new LambdaRequest(() -> isPrefire = true),
	// 			motorizedHood.angleRequest(Constants.MotorizedHood.kMidAngle),
	// 			shooter.spinUpRequest(Constants.Shooter.kMidTopRPM, Constants.Shooter.kMidBottomRPM), //1000.0 | 2000.0
	// 			new LambdaRequest(() -> LimelightProcessor.getInstance().ledOn(true))
	// 		)
	// 	);
	// }

	public void spinUpAndTrackState(double topRPM, double bottomRPM) {
		request(
			new ParallelRequest(
				turret.startVisionRequest(true)
				// shooter.holdWhenReadyRequest(topRPM, bottomRPM),
				// motorizedHood.angleRequest(Constants.MotorizedHood.kMidAngle)
			)
		);
	}

	public void spinUpAndTrackState(double topRPM, double bottomRPM, double hoodAngle) {
		request(
			new SequentialRequest(
				turret.angleRequest(180.0),
				new ParallelRequest(
					new LambdaRequest(() -> LimelightProcessor.getInstance().ledOn(true)),
					turret.startVisionRequest(true)
					// shooter.holdWhenReadyRequest(topRPM, bottomRPM),
					// motorizedHood.angleRequest(hoodAngle)
				)
			)	
		);
	}

	// public void fireState() {
	// 	request(
	// 		new SequentialRequest(
	// 			new ParallelRequest(
	// 				intake.stateRequest(Intake.State.FEEDING_INTAKE),
	// 				feeder.stateRequest(Feeder.State.FEEDING),
	// 				feeder.waitToFinishShootingRequest()
	// 			)
	// 		)	
	// 	);
	// }

	// public void testFiringState() {
	// 	request(
	// 		new SequentialRequest(
	// 			shooter.holdWhenReadyRequest(1200.0, 3400.0),
	// 			new ParallelRequest(
	// 				intake.stateRequest(Intake.State.FEEDING_INTAKE),
	// 				feeder.stateRequest(Feeder.State.FEEDING),
	// 				feeder.waitToFinishShootingRequest()
	// 			),
	// 			postShootingRequest()
	// 		)
	// 	);
	// }

	// public void deployHangerState() {
	// 	request(
	// 		new SequentialRequest(
	// 			turret.angleRequest(180.0),
	// 			new LambdaRequest(() -> hanger.fireExtender(true)),
	// 			waitRequest(0.5),
	// 			hanger.setHeightRequest(0.0, 0.1)
	// 		)
	// 	);
	// }

	// public void hangerState(double height) {
	// 	request(
	// 		hanger.setHeightRequest(height)
	// 	);
	// }
}
