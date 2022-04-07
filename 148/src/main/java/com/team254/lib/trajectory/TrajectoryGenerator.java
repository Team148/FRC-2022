package com.team254.lib.trajectory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import frc.Constants;
import frc.DriveMotionPlanner;
import frc.RobotMap;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.timing.CurvatureVelocityConstraint;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.trajectory.timing.TimingConstraint;

import edu.wpi.first.wpilibj.Timer;

public class TrajectoryGenerator {
    private static final double kMaxVelocity = 120.0;
    private static final double kMaxAccel = 120.0;
    private static final double kMaxDecel = 72.0;
    private static final double kMaxVoltage = 9.0;

    private static TrajectoryGenerator mInstance = new TrajectoryGenerator();
    private final DriveMotionPlanner mMotionPlanner;
    private TrajectorySet mTrajectorySet = null;

    public static TrajectoryGenerator getInstance() {
        return mInstance;
    }

    private TrajectoryGenerator() {
        mMotionPlanner = new DriveMotionPlanner();
    }

    public void generateTrajectories() {
        if(mTrajectorySet == null) {
        	double startTime = Timer.getFPGATimestamp();
            System.out.println("Generating trajectories...");
            mTrajectorySet = new TrajectorySet();
            System.out.println("Finished trajectory generation in: " + (Timer.getFPGATimestamp() - startTime) + " seconds");
        }
    }

    // CRITICAL POSES
    // Origin is the center of the robot when the robot is placed against the middle of the alliance station wall.
    // +x is towards the center of the field.
    // +y is to the right.
    // ALL POSES DEFINED FOR THE CASE THAT ROBOT STARTS ON LEFT! (mirrored about +x axis for RIGHT)
    // static final Pose2d autoStartingPose = new Pose2d(Constants.goalWallPose.getTranslation().translateBy(new Translation2d(0.0, 0.0)), Rotation2d.fromDegrees(180.0));
    // static final Pose2d partnerStartingPose = new Pose2d(Constants.goalCenterPose.getTranslation().translateBy(new Translation2d(0.0, 0.0)), Rotation2d.fromDegrees(180.0));
    // static final Pose2d altAutoStartingPose = new Pose2d(Constants.loadWallPose.getTranslation().translateBy(new Translation2d(0.0, 0.0)), Rotation2d.fromDegrees(180.0));

    // static final Pose2d firstTrenchIntakePose = Constants.testTrenchBallOne.transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength , 0.0)));
    // static final Pose2d thirdTrenchIntakePose = Constants.testTrenchBallThree.transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength, 0.0)));
    // static final Pose2d trenchPairIntakePose = Constants.testTrenchBallFourAndFive.transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength + 3.0, 0.0)));
    // static final Pose2d trenchOppositePairIntakePose = Constants.testEnemyTrenchBallFourAndFive.transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength, 0.0)));
    // static final Pose2d trenchCornerPose = new Pose2d(new Translation2d( Constants.kFieldLength - 207.57, 107.125), Rotation2d.fromDegrees(-25.0 + 180.0));
    
    // static final Pose2d generatorIntake1Pose = Constants.kGeneratorBall1.transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength, 0.0)));
    // static final Pose2d generatorIntake2Pose = Constants.kGeneratorBall2.transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength, 0.0)));    
    // static final Pose2d generatorIntake3Pose = Constants.kGeneratorBall3.transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength, 0.0)));    
    // static final Pose2d generatorIntake4Pose = Constants.kGeneratorBall4.transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength, 0.0)));
    // static final Pose2d generatorIntake5Pose = Constants.kGeneratorBall5.transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength, 0.0)));
    // static final Pose2d generatorIntake1and2Pose = Constants.kGeneratorBall1and2.transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength, 0.0)));
    // static final Pose2d generatorIntake3and4Pose = Constants.kGeneratorBall3and4.transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength + 4.0, -9.0)));

    // static final Pose2d enemyHumanLoaderIntakePose = Constants.kEnemyHumanLoader.transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength, 0.0)));

    public TrajectorySet getTrajectorySet() {
        return mTrajectorySet;
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_decel,
            double max_voltage,
            double default_vel,
            int slowdown_chunks) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, max_vel, max_accel, max_decel, max_voltage, 
            default_vel, slowdown_chunks);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double start_vel,  // inches/s
            double end_vel,  // inches/s
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_decel,
            double max_voltage,
            double default_vel,
            int slowdown_chunks) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, start_vel, end_vel, max_vel, max_accel, max_decel, max_voltage, 
            default_vel, slowdown_chunks);
    }

    public class TrajectorySet {
        public class MirroredTrajectory {
            public MirroredTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> left) {
                this.left = left;
                this.right = TrajectoryUtil.mirrorTimed(left, left.defaultVelocity());
            }

            public Trajectory<TimedState<Pose2dWithCurvature>> get(boolean left) {
                return left ? this.left : this.right;
            }

            public final Trajectory<TimedState<Pose2dWithCurvature>> left;
            public final Trajectory<TimedState<Pose2dWithCurvature>> right;
        }


        public final Trajectory<TimedState<Pose2dWithCurvature>> twoStartToBall;
        public final Trajectory<TimedState<Pose2dWithCurvature>> twoBallToBallTwo;
        public final Trajectory<TimedState<Pose2dWithCurvature>> twoBallsToTheWall;
        public final Trajectory<TimedState<Pose2dWithCurvature>> terminalToShot;

        public final Trajectory<TimedState<Pose2dWithCurvature>> oneBallStartToBallOne;

        public final Trajectory<TimedState<Pose2dWithCurvature>> oneBallOneToDefend;
        public final Trajectory<TimedState<Pose2dWithCurvature>> oneBallToDefendTwo;
        public final Trajectory<TimedState<Pose2dWithCurvature>> oneBallDefendToDrop;

        // public final Trajectory<TimedState<Pose2dWithCurvature>> oneBallOneToDefendPA;
        // public final Trajectory<TimedState<Pose2dWithCurvature>> oneBallToDefendTwoPA;

        public final Trajectory<TimedState<Pose2dWithCurvature>> oneBallTerminal;
        public final Trajectory<TimedState<Pose2dWithCurvature>> oneBallTerminalShot;

        public final Trajectory<TimedState<Pose2dWithCurvature>> yourPartnersBalls;
        public final Trajectory<TimedState<Pose2dWithCurvature>> yourPartnersBallsPart2;

        public final Trajectory<TimedState<Pose2dWithCurvature>> terminalToShotButBetter;



        private TrajectorySet() {
        
            twoStartToBall = getTwoStartToBall();
            twoBallToBallTwo = getTwoBallToBallTwo();
            twoBallsToTheWall = getTwoBallsToTheWall();
            terminalToShot = getTerminalToShot();

            oneBallStartToBallOne = getOneBallStartToBallOne();

            oneBallOneToDefend = getOneBallOneToDefend();
            oneBallToDefendTwo = getOneBallToDefendTwo();
            oneBallDefendToDrop = getOneBallDefendToDrop();

            // oneBallOneToDefendPA = getOneBallOneToDefendPA();
            // oneBallToDefendTwoPA = getOneBallToDefendTwoPA();

            oneBallTerminal = getOneBallTerminal();
            oneBallTerminalShot = getOneBallTerminalShot();

            yourPartnersBalls = getYourPartnersBalls();
            yourPartnersBallsPart2 = getYourPartnersBallsPart2();
            terminalToShotButBetter = getTerminalToShotButBetter();

        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTwoStartToBall(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.twoBallStart);//250
            waypoints.add(Constants.twoBallOne);//.transformBy(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(270.0)))

            return generateTrajectory(false, waypoints, Arrays.asList(), 100.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTwoBallToBallTwo(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.twoBallOne2);
            waypoints.add(Constants.twoBallTwo);

            return generateTrajectory(false, waypoints, Arrays.asList(), 100.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTwoBallsToTheWall(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.twoBallTutu);
            waypoints.add(Constants.twoBallTerminal);
            
            return generateTrajectory(false, waypoints, Arrays.asList(), 150.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTerminalToShot(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.twoBallTerminal2);
            waypoints.add(Constants.postTerminalShot);

            return generateTrajectory(false, waypoints, Arrays.asList(), 150.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getOneBallStartToBallOne(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.oneBallStart);
            waypoints.add(Constants.oneBallOne);

            return generateTrajectory(false, waypoints, Arrays.asList(), 75.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getOneBallOneToDefend(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.oneBallOne2);
            waypoints.add(Constants.oneBallOneDefend);
            
            return generateTrajectory(false, waypoints, Arrays.asList(), 100.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getOneBallToDefendTwo(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.oneBallOneDefend2);
            waypoints.add(Constants.oneBallToDefend);

            
            return generateTrajectory(false, waypoints, Arrays.asList(), 100.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getOneBallDefendToDrop(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.oneBallToDefend);
            waypoints.add(Constants.oneBallDefendDropOff);

            
            return generateTrajectory(false, waypoints, Arrays.asList(), 20.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getOneBallOneToDefendPA(){
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(Constants.oneBallOne3);
        //     waypoints.add(Constants.oneBallOneDefendPA);
            
        //     return generateTrajectory(false, waypoints, Arrays.asList(), 100.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getOneBallToDefendTwoPA(){
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(Constants.oneBallOneDefendPA2);
        //     waypoints.add(Constants.oneBallToDefendPA);

            
        //     return generateTrajectory(false, waypoints, Arrays.asList(), 100.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        // }

        private Trajectory<TimedState<Pose2dWithCurvature>> getOneBallTerminal(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.oneBallOne);
            waypoints.add(Constants.oneBallTerminal);

            
            return generateTrajectory(false, waypoints, Arrays.asList(), 20.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getOneBallTerminalShot(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.oneBallTerminal);
            waypoints.add(Constants.oneBallTerminalShot);

            
            return generateTrajectory(false, waypoints, Arrays.asList(), 20.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getYourPartnersBalls(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.oneBallStart);
            waypoints.add(Constants.oneBallAndPartnerBalls);

            
            return generateTrajectory(false, waypoints, Arrays.asList(), 20.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getYourPartnersBallsPart2(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.oneBallAndPartnerBalls2);
            waypoints.add(Constants.oneBallOne3);

            
            return generateTrajectory(false, waypoints, Arrays.asList(), 20.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        //no
        private Trajectory<TimedState<Pose2dWithCurvature>> getTerminalToShotButBetter(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.twoBallTerminal2);
            waypoints.add(Constants.oneBallTerminalShot);

            return generateTrajectory(false, waypoints, Arrays.asList(), 150.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }
    }
}
