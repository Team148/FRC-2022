package frc.auto.modes;

import java.util.ArrayList;
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
import frc.auto.actions.SetPivotState;
import frc.auto.actions.SetShooterSpeedAction;
import frc.auto.actions.SetTrajectoryAction;
import frc.auto.actions.SetTurretAngleAction;
import frc.auto.actions.SetTurretState;
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
import frc.subsystems.IntakePivot;
import frc.subsystems.Superstructure;
import frc.subsystems.Swerve;
import frc.subsystems.Hanger.HangerState;
import frc.subsystems.Feeder.FeederState;
import frc.subsystems.IntakePivot.PivotState;
import frc.subsystems.Turret.State;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryGenerator;
import com.team254.lib.trajectory.timing.TimedState;

import edu.wpi.first.wpilibj.Timer;

public class TwoBallAndTerminalBlue extends AutoModeBase {
    Superstructure s;

    @Override
    public List<Trajectory<TimedState<Pose2dWithCurvature>>> getPaths() {
        return Arrays.asList(trajectories.twoStartToBall, trajectories.twoBallToBallTwo, trajectories.twoBallsToTheWall);
    }

	public TwoBallAndTerminalBlue() {
        s = Superstructure.getInstance();
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        super.startTime = Timer.getFPGATimestamp();
        runAction(new ResetPoseAction(Constants.twoBallStart));//.transformBy(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(270.0)))
        runAction(new SetPivotState(PivotState.DOWN));
        LimelightProcessor.getInstance().ledOn(true);
        runAction(new SetFeederState(FeederState.INTAKING));
        runAction(new SetBallIntakeSpeedAction(-1.0));
        runAction(new SetTrajectoryAction(trajectories.twoStartToBall, -90.0, 1.5));
        s.turretPositionState(90.0);
        runAction(new SetShooterSpeedAction(11750.0));
        runAction(new SetHoodAngleAction(28.0));
        runAction(new RemainingProgressAction(0.05));
        s.firingVision();
        runAction(new SetFeederState(FeederState.SHOOTING));
        runAction(new WaitAction(1.75));
        runAction(new SetFeederState(FeederState.INTAKING));
        runAction(new SetTrajectoryAction(trajectories.twoBallToBallTwo, 35.0, 0.75));
            runAction(new SetShooterSpeedAction(12250.0));
            runAction(new SetHoodAngleAction(32.0));
        runAction(new SetTurretAngleAction(20.0));
        s.firingVision();
        runAction(new RemainingProgressAction(0.05));
        runAction(new SetFeederState(FeederState.SHOOTING));
        runAction(new WaitAction(1.25));
        runAction(new SetTrajectoryAction(trajectories.twoBallsToTheWall, -45.0, 0.6));
        runAction(new SetFeederState(FeederState.INTAKING));
        runAction(new RemainingProgressAction(0.01));
        runAction(new WaitAction(0.75));
        runAction(new SetTrajectoryAction(trajectories.terminalToShot, 184.0, 0.5));
        runAction(new SetTurretAngleAction(180.0));
        runAction(new SetShooterSpeedAction(11500));
        runAction(new RemainingProgressAction(0.05));
        s.firingVision();
        runAction(new SetFeederState(FeederState.SHOOTING));

        System.out.println("Auto mode finished in " + currentTime() + " seconds");
	}
}
