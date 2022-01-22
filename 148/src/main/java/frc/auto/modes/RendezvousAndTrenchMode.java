package frc.auto.modes;
import java.util.Arrays;
import java.util.List;
import frc.Constants;
import frc.RobotState;
import frc.auto.AutoModeBase;
import frc.auto.AutoModeEndedException;
import frc.auto.actions.ParallelAction;
import frc.auto.actions.RemainingProgressAction;
import frc.auto.actions.ResetPoseAction;
import frc.auto.actions.SeriesAction;
import frc.auto.actions.SetBallIntakeSpeedAction;
import frc.auto.actions.SetHoodAngleAction;
import frc.auto.actions.SetBallHandoffState;
import frc.auto.actions.SetShooterSpeedAction;
import frc.auto.actions.SuperStructureAction;
import frc.auto.actions.SetTrajectoryAction;
import frc.auto.actions.SetTurretAngleAction;
import frc.auto.actions.SetTurretState;
import frc.auto.actions.StowHanger;
import frc.auto.actions.WaitAction;
import frc.auto.actions.WaitForDistanceAction;
import frc.auto.actions.WaitForHeadingAction;
import frc.auto.actions.WaitForSuperstructureAction;
import frc.auto.actions.WaitForVisionAction;
import frc.auto.actions.WaitToFinishPathAction;
import frc.auto.actions.WaitToPassXCoordinateAction;
import frc.auto.actions.WaitToPassYCoordinateAction;
import frc.loops.LimelightProcessor;
import frc.loops.LimelightProcessor.Pipeline;
import frc.subsystems.Superstructure;
import frc.subsystems.Swerve;
import frc.subsystems.Hanger.HangerState;
import frc.subsystems.BallHandoff.BallHandoffState;
import frc.subsystems.Turret.State;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryGenerator;
import com.team254.lib.trajectory.timing.TimedState;
import edu.wpi.first.wpilibj.Timer;
public class RendezvousAndTrenchMode extends AutoModeBase {
    Superstructure s;
    @Override
    public List<Trajectory<TimedState<Pose2dWithCurvature>>> getPaths() {
        return Arrays.asList(trajectories.wallToOffset, trajectories.offsetToTrenchBallOne, trajectories.trenchBallOnceToTrenchFull,
        trajectories.trenchFullToTrenchCloseShot);
    }
    public RendezvousAndTrenchMode() {
        s = Superstructure.getInstance();
    }
    @Override
    protected void routine() throws AutoModeEndedException {
        super.startTime = Timer.getFPGATimestamp();
        LimelightProcessor.getInstance().ledOn(true);
        runAction(new ResetPoseAction(Constants.goalWallPose));
        runAction(new SetShooterSpeedAction(12500));
        s.turretPositionState(145.0);
        runAction(new StowHanger(HangerState.STOW));
        runAction(new SetHoodAngleAction(40.0));
        runAction(new WaitAction(0.5));
        runAction(new SuperStructureAction());
        runAction(new SetBallHandoffState(BallHandoffState.SHOOTING));
        runAction(new SetBallIntakeSpeedAction(0.69));
        // s.firingVision();
        runAction(new WaitAction(1.0));
        runAction(new ParallelAction (
                    Arrays.asList(
                        new SetTrajectoryAction(trajectories.wallToTrenchBallThree, 180.0, 1.0),
                        new SetHoodAngleAction(50.0),
                        new SetShooterSpeedAction(14500)
                    )
                ));
        runAction(new RemainingProgressAction(0.01));
        runAction(new WaitAction(1.7));
        //rendezvous beyond
        runAction(new SetTrajectoryAction(trajectories.toRendezvousThreeBall, 330.0, 1.0));
        runAction(new RemainingProgressAction(0.01));
        s.turretPositionState(0.0);
        runAction(new SetShooterSpeedAction(14000));
        runAction(new SetBallHandoffState(BallHandoffState.INTAKING));
        runAction(new SetTrajectoryAction(trajectories.rendezvousThreeBall, 330.0, 1.0));
        runAction(new RemainingProgressAction(0.01));
        runAction(new SetTrajectoryAction(trajectories.rendezvousThreeBall2, 330.0, 1.0));
        runAction(new RemainingProgressAction(0.01));
        runAction(new SetTrajectoryAction(trajectories.rendezvousTwoBall, 330.0, 1.0));
        runAction(new RemainingProgressAction(0.01));
        // s.firingVision();
        s.turretPositionState(180.0);
        runAction(new SetTrajectoryAction(trajectories.twoToPew, 180.0, 1.0));
        runAction(new WaitToFinishPathAction());
        s.firingVision();
        runAction(new SetBallHandoffState(BallHandoffState.SHOOTING));
        //rendezvous^^
        System.out.println("Auto mode finished in " + currentTime() + " seconds");
    }
}