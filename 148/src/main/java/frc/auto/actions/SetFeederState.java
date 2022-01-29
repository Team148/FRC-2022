package frc.auto.actions;

import frc.Robot;
import frc.subsystems.Feeder.FeederState;

public class SetFeederState extends RunOnceAction{
	FeederState desiredState;
	
	public SetFeederState(FeederState state){
		desiredState = state;
	}
	
	@Override
	public void runOnce() {
		// System.out.println("Setting FeederState!!!!!" + desiredState);
		Robot.getFeeder().setState(desiredState);
	}
}