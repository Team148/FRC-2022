package frc.auto.actions;

import frc.subsystems.FalconHood;

public class SetHoodAngleAction extends RunOnceAction{
	double angle;
	FalconHood hood;
	
	public SetHoodAngleAction(double angle){
		this.angle = angle;
		hood = FalconHood.getInstance();
	}
	
	@Override
	public void runOnce() {
		// System.out.println("Setting Hood Angle NOW!!!!!" + angle.getUnboundedDegrees());
		hood.setHoodPosition(angle);
	}
}