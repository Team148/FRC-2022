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
import frc.auto.actions.SetHoodState;
import frc.auto.actions.SetPivotState;
import frc.auto.actions.SetBallIntakeState;
import frc.auto.actions.SetFeederState;
import frc.auto.actions.SetShooterSpeedAction;
import frc.auto.actions.SetShooterState;
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
import frc.subsystems.Feeder;
import frc.subsystems.Superstructure;
import frc.subsystems.Swerve;
import frc.subsystems.BallIntake.BallIntakeState;
import frc.subsystems.FalconHood.HoodState;
import frc.subsystems.Hanger.HangerState;
import frc.subsystems.IntakePivot.PivotState;
import frc.subsystems.Shooter.ShooterState;
import frc.subsystems.Feeder.FeederState;
import frc.subsystems.Turret.State;

import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryGenerator;
import com.team254.lib.trajectory.timing.TimedState;

import edu.wpi.first.wpilibj.Timer;

public class OneBallBase extends AutoModeBase {
    Superstructure s;

    @Override
    public List<Trajectory<TimedState<Pose2dWithCurvature>>> getPaths() {
        return Arrays.asList(trajectories.oneBallStartToBallOne);
    }

	public OneBallBase() {
        s = Superstructure.getInstance();
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        super.startTime = Timer.getFPGATimestamp();
        LimelightProcessor.getInstance().ledOn(true);
        runAction(new ResetPoseAction(Constants.oneBallStart));
        runAction(new SetShooterState(ShooterState.AUTO));
        runAction(new SetHoodState(HoodState.AUTO));
        runAction(new SetPivotState(PivotState.DOWN));
        runAction(new SetFeederState(FeederState.INTAKING));
        runAction(new SetBallIntakeState(BallIntakeState.INTAKING));
        runAction(new SetShooterSpeedAction(12000.0));
        runAction(new SetHoodAngleAction(29.0));
        runAction(new SetTurretAngleAction(-187.0));
        runAction(new SetTrajectoryAction(trajectories.oneBallStartToBallOne, 45.0, 1.0));
        runAction(new RemainingProgressAction(0.05));
        s.firingVision();
        runAction(new SetFeederState(FeederState.SHOOTING));

        System.out.println("Auto mode finished in " + currentTime() + " seconds");
	}
}
