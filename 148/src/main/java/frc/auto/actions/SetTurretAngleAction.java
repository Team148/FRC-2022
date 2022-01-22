package frc.auto.actions;

import frc.subsystems.Turret;

public class SetTurretAngleAction extends RunOnceAction{
	double angle;
	Turret turret;
	
	public SetTurretAngleAction(double angle){
		this.angle = angle;
		turret = Turret.getInstance();
	}
	
	@Override
	public void runOnce() {
		turret.setPosition(angle);
	}
}