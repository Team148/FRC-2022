package frc.auto.actions;

import frc.subsystems.Swerve;

public class TurnToHeading extends RunOnceAction{
	double targetHeading;
	Swerve swerve;
	
	public TurnToHeading(double targetHeading){
		this.targetHeading = targetHeading;
		swerve = Swerve.getInstance();
	}
	
	@Override
	public void runOnce() {
		swerve.rotate(targetHeading);;
	}

}