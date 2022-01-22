// package com.team1323.frc2020.auto.modes;

// import java.util.Arrays;
// import java.util.List;

// import com.team1323.frc2020.Constants;
// import com.team1323.frc2020.auto.AutoModeBase;
// import com.team1323.frc2020.auto.AutoModeEndedException;
// import com.team1323.frc2020.auto.actions.ResetPoseAction;
// import com.team1323.frc2020.auto.actions.SetTrajectoryAction;
// import com.team1323.frc2020.auto.actions.WaitAction;
// import com.team1323.frc2020.auto.actions.WaitForSuperstructureAction;
// import com.team1323.frc2020.auto.actions.WaitToFinishPathAction;
// import com.team1323.frc2020.subsystems.ActuatingHood;
// import com.team1323.frc2020.subsystems.Hood;
// import com.team1323.frc2020.subsystems.Superstructure;
// import com.team254.lib.geometry.Pose2dWithCurvature;
// import com.team254.lib.trajectory.Trajectory;
// import com.team254.lib.trajectory.timing.TimedState;

// import edu.wpi.first.wpilibj.Timer;

// public class TrenchRunMode extends AutoModeBase {
//     Superstructure s;

//     @Override
//     public List<Trajectory<TimedState<Pose2dWithCurvature>>> getPaths() {
//         return Arrays.asList(trajectories.startToTrenchRun, trajectories.trenchRunToCorner);
//     }

//     public TrenchRunMode() {
//         s = Superstructure.getInstance();
//     }

//     @Override
//     protected void routine() throws AutoModeEndedException {
//         super.startTime = Timer.getFPGATimestamp();
//         runAction(new ResetPoseAction(Constants.kRobotStartingPose));
//         s.firingVisionState(1200.0, 3400.0);
//         runAction(new WaitForSuperstructureAction());
//         runAction(new WaitAction(2.0));
//         s.intakeState();
//         //s.actuatingHood.conformToState(ActuatingHood.State.STOWED);
//         runAction(new SetTrajectoryAction(trajectories.startToTrenchRun, 180.0, 0.75));
//         runAction(new WaitToFinishPathAction());
//         s.neutralState();
//         runAction(new SetTrajectoryAction(trajectories.trenchRunToCorner, 180.0, 1.0));
//         runAction(new WaitToFinishPathAction());
//         //s.actuatingHood.conformToState(ActuatingHood.State.FAR);
//         runAction(new WaitForSuperstructureAction());
//         s.firingVisionState(1400.0, 4200.0);
//         runAction(new WaitForSuperstructureAction());
//         System.out.println("Auto mode finished in " + currentTime() + " seconds");
//     }
// }
    