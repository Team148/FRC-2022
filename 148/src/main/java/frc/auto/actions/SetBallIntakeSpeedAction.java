package frc.auto.actions;

import frc.subsystems.BallIntake;

public class SetBallIntakeSpeedAction extends RunOnceAction{
	double speed;
	BallIntake ballIntake;
	
	public SetBallIntakeSpeedAction(double speed){
		this.speed = speed;
		ballIntake = BallIntake.getInstance();
	}
	
	@Override
	public void runOnce() {
		ballIntake.setMotor(speed);
	}
}