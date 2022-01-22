package frc.auto.actions;

import frc.subsystems.Swerve;
import com.team254.lib.geometry.Rotation2d;

/**
 * 
 */
public class WaitForTrajectoryDirectionAction implements Action{
    Rotation2d lowerBound, upperBound;
    Swerve swerve;
    public WaitForTrajectoryDirectionAction(Rotation2d lower, Rotation2d upper){
        lowerBound = lower;
        upperBound = upper;
        swerve = Swerve.getInstance();
    }

    @Override
    public boolean isFinished() {
        double direction = swerve.getLastTrajectoryVector().direction().getDegrees();
        return lowerBound.getDegrees() <= direction && direction <= upperBound.getDegrees();
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {

    }

    @Override
    public void done() {

    }
}
