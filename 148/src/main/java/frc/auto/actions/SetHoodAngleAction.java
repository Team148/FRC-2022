package frc.auto.actions;

import frc.subsystems.MotorizedHood;

public class SetHoodAngleAction extends RunOnceAction{
	double angle;
	MotorizedHood hood;
	
	public SetHoodAngleAction(double angle){
		this.angle = angle;
		hood = MotorizedHood.getInstance();
	}
	
	@Override
	public void runOnce() {
		// System.out.println("Setting Hood Angle NOW!!!!!" + angle.getUnboundedDegrees());
		hood.setAngle(angle);
	}
}