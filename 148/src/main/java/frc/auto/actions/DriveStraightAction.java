package frc.auto.actions;

import frc.subsystems.Swerve;
import com.team254.lib.geometry.Translation2d;

public class DriveStraightAction extends RunOnceAction{
	Translation2d driveVector;

	public DriveStraightAction(Translation2d driveVector){
		this.driveVector = driveVector;
	}

	@Override
	public void runOnce() {
		Swerve.getInstance().sendInput(driveVector.x(), driveVector.y(), 0.0, false, false);
	}
	
}
