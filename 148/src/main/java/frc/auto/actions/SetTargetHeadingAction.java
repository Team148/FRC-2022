package frc.auto.actions;

import frc.subsystems.Swerve;

public class SetTargetHeadingAction extends RunOnceAction{
	double targetHeading;
	Swerve swerve;
	
	public SetTargetHeadingAction(double targetHeading){
		this.targetHeading = targetHeading;
		swerve = Swerve.getInstance();
	}
	
	@Override
	public void runOnce() {
		swerve.setAbsolutePathHeading(targetHeading);
	}

}
