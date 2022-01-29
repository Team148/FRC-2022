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
import frc.auto.actions.SetFeederState;
import frc.auto.actions.SetShooterSpeedAction;
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
import frc.subsystems.Feeder.FeederState;
import frc.subsystems.Turret.State;

import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryGenerator;
import com.team254.lib.trajectory.timing.TimedState;

import edu.wpi.first.wpilibj.Timer;

public class ThiefAndRendezvous extends AutoModeBase {
    Superstructure s;

    @Override
    public List<Trajectory<TimedState<Pose2dWithCurvature>>> getPaths() {
        return Arrays.asList(trajectories.loadWallToEnemyTrenchBackward, trajectories.enemyTrenchToGoalOffsetBackward, trajectories.reverseRendezvousTwoBall,
        trajectories.reverseRendezvousToThreeBall, trajectories.reverseRendezvousThreeBall);
    }

	public ThiefAndRendezvous() {
        s = Superstructure.getInstance();
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        super.startTime = Timer.getFPGATimestamp();
        LimelightProcessor.getInstance().ledOn(true);
        runAction(new ResetPoseAction(Constants.loadWallPose));
        runAction(new SetShooterSpeedAction(14500));
        runAction(new StowHanger(HangerState.STOW));
        runAction(new SetFeederState(FeederState.INTAKING));
        runAction(new SetBallIntakeSpeedAction(0.69));
        s.turretPositionState(200.0);
        runAction(new SetTrajectoryAction(trajectories.loadWallToEnemyTrench, 180.0, 1.0));
        runAction(new RemainingProgressAction(0.01));
        runAction(new SetTrajectoryAction(trajectories.enemyTrenchToGoalOffsetLeft, 270.0, 1.0));
        s.firingVision();
        runAction(new SetHoodAngleAction(46.0));
        runAction(new WaitToFinishPathAction());
        runAction(new SetFeederState(FeederState.SHOOTING));
        runAction(new WaitAction(2.5));
        //rendezvous beyond
        runAction(new SetFeederState(FeederState.INTAKING));
        s.turretPositionState(0.0);
        runAction(new SetShooterSpeedAction(12500));
        runAction(new SetTrajectoryAction(trajectories.toReverseRendezvousTwoBall, 180.0, 1.0));
        runAction(new RemainingProgressAction(0.05));
        runAction(new SetTrajectoryAction(trajectories.reverseRendezvousTwoBall, 180.0, 1.0));
        runAction(new WaitToFinishPathAction());
        s.turretPositionState(0.0);
        runAction(new SetTrajectoryAction(trajectories.reverseRendezvousToThreeBall, 0.0, 1.0));
        runAction(new WaitToFinishPathAction());
        runAction(new SetTrajectoryAction(trajectories.reverseRendezvousThreeBall, 0.0, 1.0));
        s.turretPositionState(0.0);
        runAction(new RemainingProgressAction(0.01));
        runAction(new SetTrajectoryAction(trajectories.threeToShoot, 0.0, 1.0));
        runAction(new WaitToFinishPathAction());
        s.firingVision();
        runAction(new SetFeederState(FeederState.SHOOTING)); 
        //rendezvous ^^

        System.out.println("Auto mode finished in " + currentTime() + " seconds");
	}
}
