package frc.auto.actions;

import frc.Robot;
import frc.subsystems.BallIntake.BallIntakeState;

public class SetBallIntakeState extends RunOnceAction{
	BallIntakeState desiredState;
	
	public SetBallIntakeState(BallIntakeState state){
		desiredState = state;
	}
	
	@Override
	public void runOnce() {
		// System.out.println("Setting FeederState!!!!!" + desiredState);
		Robot.getBallIntake().setState(desiredState);
	}
}
