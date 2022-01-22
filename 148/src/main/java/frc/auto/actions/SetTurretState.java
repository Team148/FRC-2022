package frc.auto.actions;

import frc.Robot;
import frc.subsystems.Turret.State;

public class SetTurretState extends RunOnceAction{
	State desiredState;
	
	
	public SetTurretState(State state){
		desiredState = state;
	}
	
	@Override
	public void runOnce() {
		// System.out.println("Setting the TurretState!!!!!" + desiredState.name());
		Robot.getTurret().setState(desiredState);
	}
}