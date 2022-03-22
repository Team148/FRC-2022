package frc.auto.actions;

import frc.Robot;
import frc.subsystems.Shooter.ShooterState;

public class SetShooterState extends RunOnceAction{
	ShooterState desiredState;
	
	public SetShooterState(ShooterState state){
		desiredState = state;
	}
	
	@Override
	public void runOnce() {
		// System.out.println("Setting FeederState!!!!!" + desiredState);
		Robot.getShooter().setState(desiredState);
	}
}