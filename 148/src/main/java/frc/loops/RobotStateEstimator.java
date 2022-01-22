package frc.loops;

import frc.RobotState;
import frc.subsystems.Swerve;
import frc.subsystems.Turret;

public class RobotStateEstimator implements Loop{
	private static RobotStateEstimator instance = null;
	public static RobotStateEstimator getInstance(){
		if(instance == null)
			instance = new RobotStateEstimator();
		return instance;
	}
	
	RobotStateEstimator(){
	}
	
	RobotState robotState = RobotState.getInstance();
	Swerve swerve;
	Turret turret;

	@Override
	public void onStart(double timestamp) {
		swerve = Swerve.getInstance();
		turret = Turret.getInstance();
	}

	@Override
	public void onLoop(double timestamp) {
		robotState.addObservations(timestamp, swerve.getPose(), swerve.getVelocity(), turret.getRotation());
	}

	@Override
	public void onStop(double timestamp) {
		
	}

}
