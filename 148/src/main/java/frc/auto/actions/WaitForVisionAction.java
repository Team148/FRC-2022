package frc.auto.actions;
import frc.RobotState;
import edu.wpi.first.wpilibj.Timer;

/**
 * Add your docs here.
 */
public class WaitForVisionAction implements Action {
    RobotState robotState;
    double timeout;
    double startTime = 0;

    public WaitForVisionAction(double timeout){
        robotState = RobotState.getInstance();
        this.timeout = timeout;
    }

    @Override
    public boolean isFinished() {
        return robotState.seesTarget() || (Timer.getFPGATimestamp() - startTime) > timeout;
    }

    @Override
    public void start() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {

    }
}
