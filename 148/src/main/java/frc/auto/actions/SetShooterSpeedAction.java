package frc.auto.actions;

import frc.subsystems.Shooter;

public class SetShooterSpeedAction extends RunOnceAction{
	double speed;
	Shooter shooter;
	
	public SetShooterSpeedAction(double speed){
		this.speed = speed;
		shooter = Shooter.getInstance();
	}
	
	@Override
	public void runOnce() {
		shooter.setVelocity(speed);
	}
}