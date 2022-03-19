package frc.auto.actions;

import frc.Robot;
import frc.subsystems.FalconHood.HoodState;

public class SetHoodState extends RunOnceAction{
	HoodState desiredState;
	
	public SetHoodState(HoodState state){
		desiredState = state;
	}
	
	@Override
	public void runOnce() {
		// System.out.println("Setting FeederState!!!!!" + desiredState);
		Robot.getFalconHood().setState(desiredState);
	}
}