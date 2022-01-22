package frc.auto.actions;

import frc.Robot;
import frc.subsystems.BallHandoff.BallHandoffState;

public class SetBallHandoffState extends RunOnceAction{
	BallHandoffState desiredState;
	
	public SetBallHandoffState(BallHandoffState state){
		desiredState = state;
	}
	
	@Override
	public void runOnce() {
		// System.out.println("Setting BallHandoffState!!!!!" + desiredState);
		Robot.getBallHandoff().setState(desiredState);
	}
}