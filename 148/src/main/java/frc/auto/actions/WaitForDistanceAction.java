package frc.auto.actions;

import frc.subsystems.Swerve;
import com.team254.lib.geometry.Translation2d;

/**
 * 
 */
public class WaitForDistanceAction implements Action{
    Swerve swerve;
    Translation2d pointOfInterest;
    double distance;

    public WaitForDistanceAction(Translation2d target, double distance){
        swerve = Swerve.getInstance();
        pointOfInterest = target;
        this.distance = distance;
    }

    @Override
    public boolean isFinished() {
        return swerve.getPose().getTranslation().distance(pointOfInterest) <= distance;
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
