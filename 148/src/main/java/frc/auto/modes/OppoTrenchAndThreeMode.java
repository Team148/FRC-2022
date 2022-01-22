// package com.team1323.frc2020.auto.modes;

// import java.util.Arrays;
// import java.util.List;

// import com.team1323.frc2020.Constants;
// import com.team1323.frc2020.auto.AutoModeBase;
// import com.team1323.frc2020.auto.AutoModeEndedException;
// import com.team1323.frc2020.auto.actions.RemainingProgressAction;
// import com.team1323.frc2020.auto.actions.ResetPoseAction;
// import com.team1323.frc2020.auto.actions.SetTrajectoryAction;
// import com.team1323.frc2020.auto.actions.WaitAction;
// import com.team1323.frc2020.auto.actions.WaitForSuperstructureAction;
// import com.team1323.frc2020.auto.actions.WaitToFinishPathAction;
// import com.team1323.frc2020.loops.LimelightProcessor;
// import com.team1323.frc2020.subsystems.Intake;
// import com.team1323.frc2020.subsystems.Superstructure;
// import com.team254.lib.geometry.Pose2dWithCurvature;
// import com.team254.lib.trajectory.Trajectory;
// import com.team254.lib.trajectory.timing.TimedState;

// import edu.wpi.first.wpilibj.Timer;

// public class OppoTrenchAndThreeMode extends AutoModeBase {

//     Superstructure s;

//     @Override
//     public List<Trajectory<TimedState<Pose2dWithCurvature>>> getPaths() {
//         return Arrays.asList(trajectories.altStartToOppoTrench, trajectories.oppoTrenchToStart, trajectories.startToGenerator3and4, trajectories.generator3and4ToGenerator5, /*trajectories.startToGenerator5,*/
//             trajectories.generator5ToStart/*, trajectories.startToEnemyHumanLoader*/);
//     }

//     public OppoTrenchAndThreeMode() {
//         s = Superstructure.getInstance();
//     }

//     @Override
//     protected void routine() throws AutoModeEndedException {
//         super.startTime = Timer.getFPGATimestamp();
//         LimelightProcessor.getInstance().ledOn(true);
//         runAction(new ResetPoseAction(Constants.kAltRobotStartingPose));
//         s.intakeState();
//         s.turret.setPosition(180.0);
//         s.spinUpAndTrackState(Constants.Shooter.kMidTopRPM, Constants.Shooter.kMidBottomRPM, Constants.MotorizedHood.kMidAngle);
//         runAction(new SetTrajectoryAction(trajectories.altStartToOppoTrench, 180.0, 0.75));
//         runAction(new WaitToFinishPathAction());
//         s.spinUpAndTrackState(Constants.Shooter.kMidTopRPM, Constants.Shooter.kMidBottomRPM, Constants.MotorizedHood.kMidAngle);
//         runAction(new SetTrajectoryAction(trajectories.oppoTrenchToStart, 180.0, 0.75));
//         runAction(new WaitToFinishPathAction());
//         s.intake.conformToState(Intake.State.OFF);
//         s.fireState();
//         runAction(new WaitAction(2.0));
//         s.autoIntakeState();
//         runAction(new WaitForSuperstructureAction());
//         s.spinUpAndTrackState(Constants.Shooter.kMidTopRPM, Constants.Shooter.kMidBottomRPM, Constants.MotorizedHood.kMidAngle);
//         runAction(new SetTrajectoryAction(trajectories.startToGenerator3and4, -22.5 + 180.0, 0.75));
//         runAction(new WaitToFinishPathAction());
//         runAction(new SetTrajectoryAction(trajectories.generator3and4ToGenerator5, 180.0 + 15.0, 0.75));
//         runAction(new WaitToFinishPathAction());
//         runAction(new SetTrajectoryAction(trajectories.generator5ToStart, 180.0 + 15.0, 0.75));
//         runAction(new WaitToFinishPathAction());
//         s.intake.conformToState(Intake.State.OFF);
//         s.spinUpAndTrackState(Constants.Shooter.kMidTopRPM, Constants.Shooter.kMidBottomRPM, Constants.MotorizedHood.kMidAngle);
//         runAction(new SetTrajectoryAction(trajectories.generator5ToStart, 180.0, 0.75));
//         runAction(new WaitToFinishPathAction());
//         runAction(new WaitForSuperstructureAction());
//         s.fireState();
//         System.out.println("Auto mode finished in " + currentTime() + " seconds");
//     }
    
// }