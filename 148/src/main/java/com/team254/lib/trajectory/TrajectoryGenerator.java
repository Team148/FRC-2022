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

        // //Test Paths
        public final Trajectory<TimedState<Pose2dWithCurvature>> straightPath;
        public final Trajectory<TimedState<Pose2dWithCurvature>> wallToOffset;
        public final Trajectory<TimedState<Pose2dWithCurvature>> offsetToTrenchBallOne;
        public final Trajectory<TimedState<Pose2dWithCurvature>> trenchBallOnceToTrenchFull;
        public final Trajectory<TimedState<Pose2dWithCurvature>> trenchBallOnceToTrenchFarShot;
        public final Trajectory<TimedState<Pose2dWithCurvature>> trenchBallFourToTrenchBallFive;
        public final Trajectory<TimedState<Pose2dWithCurvature>> trenchFullToTrenchCloseShot;
        public final Trajectory<TimedState<Pose2dWithCurvature>> offsetToTrenchFull;
        public final Trajectory<TimedState<Pose2dWithCurvature>> toRendezvousThreeBall;
        public final Trajectory<TimedState<Pose2dWithCurvature>> rendezvousThreeBall;
        public final Trajectory<TimedState<Pose2dWithCurvature>> rendezvousThreeBall2;
        public final Trajectory<TimedState<Pose2dWithCurvature>> rendezvousTwoBall;
        public final Trajectory<TimedState<Pose2dWithCurvature>> twoToPew;
        public final Trajectory<TimedState<Pose2dWithCurvature>> toReverseRendezvousTwoBall;
        public final Trajectory<TimedState<Pose2dWithCurvature>> reverseRendezvousTwoBall;
        public final Trajectory<TimedState<Pose2dWithCurvature>> reverseRendezvousToThreeBall;
        public final Trajectory<TimedState<Pose2dWithCurvature>> reverseRendezvousThreeBall;
        public final Trajectory<TimedState<Pose2dWithCurvature>> threeToShoot;
        public final Trajectory<TimedState<Pose2dWithCurvature>> offsetToCloserRZone;        
        public final Trajectory<TimedState<Pose2dWithCurvature>> offsetToRZoneOffset;  
        public final Trajectory<TimedState<Pose2dWithCurvature>> rZoneOffsetToCloserRZone;  
        public final Trajectory<TimedState<Pose2dWithCurvature>> closerRZoneToNearTrench;
        public final Trajectory<TimedState<Pose2dWithCurvature>> goalCenterOffsetLeftToFarRZone;
        public final Trajectory<TimedState<Pose2dWithCurvature>> rZoneLeftToBallOne;
        public final Trajectory<TimedState<Pose2dWithCurvature>> rZoneBallOneToInside;
        public final Trajectory<TimedState<Pose2dWithCurvature>> rZoneInsideToNearTrenchGoal;
        public final Trajectory<TimedState<Pose2dWithCurvature>> loadWallToEnemyTrench;
        public final Trajectory<TimedState<Pose2dWithCurvature>> enemyTrenchToGoalOffsetLeft;
        public final Trajectory<TimedState<Pose2dWithCurvature>> backPocketOffsetToEnemyRZone;
        public final Trajectory<TimedState<Pose2dWithCurvature>> wallToRZoneTwoBall;
        public final Trajectory<TimedState<Pose2dWithCurvature>> rZoneTwoBallToTrenchThreeBall;
        public final Trajectory<TimedState<Pose2dWithCurvature>> trenchThreeBallToTrenchCloseShot;
        public final Trajectory<TimedState<Pose2dWithCurvature>> rZoneTwoBallToRZoneThreeBall;
        public final Trajectory<TimedState<Pose2dWithCurvature>> rZoneBallRun;
        public final Trajectory<TimedState<Pose2dWithCurvature>> rZoneBallRunToGoalOffsetLeft;
        
        public final Trajectory<TimedState<Pose2dWithCurvature>> wallToOffsetBackward;
        public final Trajectory<TimedState<Pose2dWithCurvature>> wallToTrenchBallThree;
        public final Trajectory<TimedState<Pose2dWithCurvature>> offsetToTrenchBallOneBackward;
        public final Trajectory<TimedState<Pose2dWithCurvature>> trenchBallOneToTrenchFarShotBackward;
        public final Trajectory<TimedState<Pose2dWithCurvature>> trenchFullToTrenchCloseShotBackward;
        public final Trajectory<TimedState<Pose2dWithCurvature>> loadWallToEnemyTrenchBackward;
        public final Trajectory<TimedState<Pose2dWithCurvature>> enemyTrenchToGoalOffsetBackward;
        public final Trajectory<TimedState<Pose2dWithCurvature>> wallToRZoneTwoBallBackward;
        public final Trajectory<TimedState<Pose2dWithCurvature>> rZoneTwoBallToNearTrenchBackward;
        public final Trajectory<TimedState<Pose2dWithCurvature>> nearTrenchToTrenchBallThreeBackward;
        public final Trajectory<TimedState<Pose2dWithCurvature>> trenchThreeBallToTrenchCloseShotBackward;
        public final Trajectory<TimedState<Pose2dWithCurvature>> rZoneTwoBallToRZoneOffsetBackward;
        public final Trajectory<TimedState<Pose2dWithCurvature>> rZoneOffsetToRZoneThreeBallBackward;
        public final Trajectory<TimedState<Pose2dWithCurvature>> rZoneBallRunBackward;
        public final Trajectory<TimedState<Pose2dWithCurvature>> rZoneBallRunToGoalOffsetLeftBackward;
        public final Trajectory<TimedState<Pose2dWithCurvature>> squigly;

        public final Trajectory<TimedState<Pose2dWithCurvature>> twoStartToBall;
        public final Trajectory<TimedState<Pose2dWithCurvature>> twoBallToBallTwo;
        public final Trajectory<TimedState<Pose2dWithCurvature>> twoBallsToTheWall;

        public final Trajectory<TimedState<Pose2dWithCurvature>> oneBallStartToBallOne;

        private TrajectorySet() {
            // //Test Paths
            straightPath = getStraightPath();
            wallToOffset = getWallToOffset();
            offsetToTrenchBallOne = getOffsetToTrenchBallOne();
            trenchBallOnceToTrenchFull = getTrenchBallOneToTrenchFull();
            trenchBallOnceToTrenchFarShot = getTrenchBallOneToTrenchFarShot();
            trenchBallFourToTrenchBallFive = getTrenchBallFourToTrenchBallFive();
            trenchFullToTrenchCloseShot = getTrenchFullToTrenchCloseShot();
            offsetToTrenchFull = getOffsetToTrenchFull();
            toRendezvousThreeBall = getToRendezvousThreeBall();
            rendezvousThreeBall = getRendezvousThreeBall();
            rendezvousThreeBall2 = getRendezvousThreeBall2();
            rendezvousTwoBall = getRendezvousTwoBall();
            twoToPew = getTwoToPew();
            toReverseRendezvousTwoBall = getToReverseRendezvousTwoBall();
            reverseRendezvousTwoBall = getReverseRendezvousTwoBall();
            reverseRendezvousToThreeBall = getReverseRendezvousToThreeBall();
            reverseRendezvousThreeBall = getReverseRendezvousThreeBall();
            threeToShoot = getThreeToShoot();
            offsetToCloserRZone = getOffsetToCloserRZone();
            offsetToRZoneOffset = getOffsetToRZoneOffset();
            rZoneOffsetToCloserRZone = getRZoneOffsetToCloserRZone();
            closerRZoneToNearTrench = getCloserRZoneToNearTrench();
            goalCenterOffsetLeftToFarRZone = getGoalCenterOffsetLeftToFarRZone();
            rZoneLeftToBallOne = getRZoneLeftToBallOne();
            rZoneBallOneToInside = getRZoneBallOneToInside();
            rZoneInsideToNearTrenchGoal = getRZoneInsideToNearTrenchGoal();
            loadWallToEnemyTrench = getLoadWallToEnemyTrench();
            enemyTrenchToGoalOffsetLeft = getEnemyTrenchToGoalOffsetLeft();
            backPocketOffsetToEnemyRZone = getBackPocketOffsetToEnemyRZone();
            wallToRZoneTwoBall = getWallToRZoneTwoBall();
            rZoneTwoBallToTrenchThreeBall = getRZoneTwoBallToTrenchThreeBall();
            trenchThreeBallToTrenchCloseShot = getTrenchThreeBallToTrenchCloseShot();
            rZoneTwoBallToRZoneThreeBall = getRZoneTwoBallToRZoneThreeBall();
            rZoneBallRun = getRZoneBallRun();
            rZoneBallRunToGoalOffsetLeft = getRZoneBallRunToGoalOffsetLeft();
            
            wallToOffsetBackward = getWallToOffsetBackward();
            wallToTrenchBallThree = getWallToTrenchBallThree();
            offsetToTrenchBallOneBackward = getOffsetToTrenchBallOneBackward();
            trenchBallOneToTrenchFarShotBackward = getTrenchBallOneToTrenchFarShotBackward();
            trenchFullToTrenchCloseShotBackward = getTrenchFullToTrenchCloseShotBackward();
            loadWallToEnemyTrenchBackward = getLoadWallToEnemyTrenchBackward();
            enemyTrenchToGoalOffsetBackward = getEnemyTrenchToGoalOffsetLeftBackward();
            wallToRZoneTwoBallBackward = getWallToRZoneTwoBallBackward();
            rZoneTwoBallToNearTrenchBackward = getRZoneTwoBallToNearTrenchBackward();
            nearTrenchToTrenchBallThreeBackward = getNearTrenchToTrenchBallThreeBackward();
            trenchThreeBallToTrenchCloseShotBackward = getTrenchThreeBallToTrenchCloseShotBackward();
            rZoneTwoBallToRZoneOffsetBackward = getRZoneTwoBallToRZoneOffsetBackward();
            rZoneOffsetToRZoneThreeBallBackward = getRZoneOffsetToRZoneThreeBallBackward();
            rZoneBallRunBackward = getRZoneBallRunBackward();
            rZoneBallRunToGoalOffsetLeftBackward = getRZoneBallRunToGoalOffsetLeftBackward();
            squigly = getSquigly();

            twoStartToBall = getTwoStartToBall();
            twoBallToBallTwo = getTwoBallToBallTwo();
            twoBallsToTheWall = getTwoBallsToTheWall();
            oneBallStartToBallOne = getOneBallStartToBallOne();

        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTwoStartToBall(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.twoBallStart);
            waypoints.add(Constants.twoBallOne);

            return generateTrajectory(false, waypoints, Arrays.asList(), 20.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTwoBallToBallTwo(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.twoBallOne);
            waypoints.add(Constants.twoBallTwo);

            return generateTrajectory(false, waypoints, Arrays.asList(), 20.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTwoBallsToTheWall(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.twoBallTwo);
            waypoints.add(Constants.twoBallTerminal);
            
            return generateTrajectory(false, waypoints, Arrays.asList(), 20.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getOneBallStartToBallOne(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.oneBallStart);
            waypoints.add(Constants.oneBallOne);

            return generateTrajectory(false, waypoints, Arrays.asList(), 20.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getStraightPath(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(505.0, -70.0), new Rotation2d(0.0)));
            waypoints.add(new Pose2d(new Translation2d(430.0, -70.0), new Rotation2d(0.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(), 50.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getWallToOffset(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.goalWallPose);
            waypoints.add(Constants.goalCenterOffsetPose);
            // waypoints.add(new Pose2d(new Translation2d(495.0, -142.0), new Rotation2d(0.0)));
            // waypoints.add(new Pose2d(new Translation2d(495.0, -110.0), new Rotation2d(0.0)));
            return generateTrajectory(true, waypoints, Arrays.asList(), 50.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getOffsetToTrenchBallOne(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.goalCenterOffsetPose);
            waypoints.add(Constants.trenchBallOnePose);
            return generateTrajectory(true, waypoints, Arrays.asList(), 0.0, 35.0, 35.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 15);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTrenchBallOneToTrenchFull(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.trenchBallOnePose);
            waypoints.add(Constants.trenchBallTwoPose);
            waypoints.add(Constants.trenchBallThreePose);
            waypoints.add(Constants.trenchBallFourPose);
            // waypoints.add(trenchBallFourPose.transformBy(new Pose2d(new Translation2d(0.0,-5.0), new Rotation2d(0.0))));
            return generateTrajectory(true, waypoints, Arrays.asList(), 35.0, 0.0, 100.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 30);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTrenchBallOneToTrenchFarShot(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.trenchBallOnePose);
            waypoints.add(Constants.trenchBallTwoPose);
            waypoints.add(Constants.trenchBallThreePose);
            waypoints.add(Constants.trenchBallFourPose.transformBy(new Pose2d(new Translation2d(-15.0,0.0), new Rotation2d(0.0))));
            // waypoints.add(trenchBallFourPose.transformBy(new Pose2d(new Translation2d(0.0,-5.0), new Rotation2d(0.0))));
            return generateTrajectory(true, waypoints, Arrays.asList(), 35.0, 0.0, 100.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 30);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTrenchBallFourToTrenchBallFive(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.trenchBallFourPose.transformBy(new Pose2d(new Translation2d(0.0,0.0), new Rotation2d(90.0))));
            waypoints.add(Constants.trenchBallFourPose.transformBy(new Pose2d(new Translation2d(0.0,-5.0), new Rotation2d(90.0))));
            return generateTrajectory(false, waypoints, Arrays.asList(), 35.0, 0.0, 100.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 30);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTrenchFullToTrenchCloseShot(){
            List<Pose2d> waypoints = new ArrayList<>();
            // waypoints.add(trenchBallFourPose.transformBy(new Pose2d(new Translation2d(0.0,-5.0), new Rotation2d(0.0))));
            waypoints.add(Constants.trenchBallFourPose);
            waypoints.add(Constants.trenchBallThreePose);
            waypoints.add(Constants.trenchBallTwoPose);
            // waypoints.add(trenchBallOnePose);
            return generateTrajectory(false, waypoints, Arrays.asList(), 100.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 15);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getOffsetToTrenchFull(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.goalCenterOffsetPose);
            waypoints.add(Constants.trenchBallOnePose);
            waypoints.add(Constants.trenchBallTwoPose);
            // waypoints.add(trenchBallTwoPose);
            waypoints.add(Constants.trenchBallThreePose);
            waypoints.add(Constants.trenchBallFourPose);
            return generateTrajectory(true, waypoints, Arrays.asList(), 75.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 15);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getToRendezvousThreeBall(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.testTrenchBallThree);
            waypoints.add(new Pose2d(new Translation2d(Constants.kFieldLength - (135.0 + 182.0), (Constants.kFieldHalfWidth - 83.0)), new Rotation2d(180.0)));
            waypoints.add(new Pose2d(new Translation2d(Constants.kFieldLength - (135.0 + 182.0), (Constants.kFieldHalfWidth - 125.0)), new Rotation2d(180.0)));
            return generateTrajectory(false, waypoints, Arrays.asList(), 75.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 15);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRendezvousThreeBall(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(Constants.kFieldLength - (135.0 + 182.0), (Constants.kFieldHalfWidth - 125.0)), new Rotation2d(330.0)));//y+5 through three
            waypoints.add(new Pose2d(new Translation2d(Constants.kFieldLength - (135.0 + 144.0), (Constants.kFieldHalfWidth - 132.0)), Rotation2d.fromDegrees(330.0)));
            waypoints.add(new Pose2d(new Translation2d(Constants.kFieldLength - (135.0 + 123.0), (Constants.kFieldHalfWidth - 150.0)), Rotation2d.fromDegrees(330.0)));
            return generateTrajectory(false, waypoints, Arrays.asList(), 75.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 15);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRendezvousThreeBall2(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(Constants.kFieldLength - (135.0 + 123.0), (Constants.kFieldHalfWidth - 150.0)), Rotation2d.fromDegrees(150.0)));
            waypoints.add(new Pose2d(new Translation2d(Constants.kFieldLength - (135.0 + 180.0), (Constants.kFieldHalfWidth - 125.0)), new Rotation2d(150.0)));
            return generateTrajectory(false, waypoints, Arrays.asList(), 100.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 15);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRendezvousTwoBall(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(Constants.kFieldLength - (135.0 + 205.0), (Constants.kFieldHalfWidth - 130.0)), new Rotation2d(330.0))); //y+10 through two
            waypoints.add(new Pose2d(new Translation2d(Constants.kFieldLength - (135.0 + 140.0), -(Constants.kFieldHalfWidth - 148.0)), Rotation2d.fromDegrees(330.0)));
            waypoints.add(new Pose2d(new Translation2d(Constants.kFieldLength - (135.0 + 127.0), -(Constants.kFieldHalfWidth - 148.0)), Rotation2d.fromDegrees(330.0)));
            return generateTrajectory(false, waypoints, Arrays.asList(), 100.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 15);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTwoToPew(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(Constants.kFieldLength - (135.0 + 127.0), -(Constants.kFieldHalfWidth - 148.0)), Rotation2d.fromDegrees(90.0)));
            //waypoints.add(new Pose2d(new Translation2d(Constants.kFieldLength - (135.0 + 60.0), -(Constants.kFieldHalfWidth - 135.0)), Rotation2d.fromDegrees(0.0)));
            //waypoints.add(new Pose2d(new Translation2d(Constants.kFieldLength - (135.0 + 44.0), (Constants.kFieldHalfWidth - 136.0)), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(new Translation2d(Constants.kFieldLength - (135.0 + 140.0), (Constants.kFieldHalfWidth - 91.0)), Rotation2d.fromDegrees(90.0)));
            return generateTrajectory(false, waypoints, Arrays.asList(), 100.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 15);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getToReverseRendezvousTwoBall(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(Constants.goalCenterOffsetLeftPose.getTranslation(), Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(new Translation2d(Constants.kFieldLength - (135.0 + 70.0), -(Constants.kFieldHalfWidth - 159.0)), Rotation2d.fromDegrees(180.0)));
            return generateTrajectory(false, waypoints, Arrays.asList(), 75.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 15);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getReverseRendezvousTwoBall(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(Constants.kFieldLength - (135.0 + 70.0), -(Constants.kFieldHalfWidth - 159.0)), Rotation2d.fromDegrees(150.0)));
            waypoints.add(new Pose2d(new Translation2d(Constants.kFieldLength - (135.0 + 150.0), (Constants.kFieldHalfWidth - 147.0)), Rotation2d.fromDegrees(180.0)));
            // waypoints.add(new Pose2d(Constants.testRZoneBall5.getTranslation(), Rotation2d.fromDegrees(150.0)));
            return generateTrajectory(false, waypoints, Arrays.asList(), 100.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 15);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getReverseRendezvousToThreeBall(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(Constants.kFieldLength - (135.0 + 150.0), (Constants.kFieldHalfWidth - 155.0)), Rotation2d.fromDegrees(180.0)));
            // waypoints.add(Constants.testRZoneBall5);
            waypoints.add(new Pose2d(new Translation2d(Constants.kFieldLength - (135.0 + 200.0), (Constants.kFieldHalfWidth - 120.0)), Rotation2d.fromDegrees(180.0)));
            return generateTrajectory(false, waypoints, Arrays.asList(), 50.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 15);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getReverseRendezvousThreeBall(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(Constants.kFieldLength - (135.0 + 200.0), (Constants.kFieldHalfWidth - 125.0)), Rotation2d.fromDegrees(330.0)));
            waypoints.add(new Pose2d(new Translation2d(Constants.kFieldLength - (135.0 + 160.0), (Constants.kFieldHalfWidth - 120.0)), Rotation2d.fromDegrees(330.0)));
            // waypoints.add(new Pose2d(new Translation2d(Constants.kFieldLength - (135 + 70), (Constants.kFieldHalfWidth - 169.0)), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(new Translation2d(Constants.kFieldLength - (135.0 + 150.0), (Constants.kFieldHalfWidth - 120.0)), Rotation2d.fromDegrees(330.0)));
            return generateTrajectory(false, waypoints, Arrays.asList(), 50.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 15);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getThreeToShoot(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(Constants.kFieldLength - (135.0 + 150.0), (Constants.kFieldHalfWidth - 120.0)), Rotation2d.fromDegrees(90.0)));
            waypoints.add(new Pose2d(new Translation2d(Constants.kFieldLength - (135.0 + 150.0), (Constants.kFieldHalfWidth - 71.0)), Rotation2d.fromDegrees(90.0)));
            return generateTrajectory(false, waypoints, Arrays.asList(), 100.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 15);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getOffsetToCloserRZone(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.goalCenterOffsetPose);
            waypoints.add(new Pose2d(Constants.goalCenterOffsetPose.transformBy(new Pose2d( new Translation2d(-70.0, 0.0), new Rotation2d(0)))));
            waypoints.add(Constants.rZoneBallOnePose.transformBy(new Pose2d( new Translation2d(5.0, -2.0), new Rotation2d(0))));
            // waypoints.add(trenchBallTwoPose);
            return generateTrajectory(true, waypoints, Arrays.asList(), 25.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 15);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getOffsetToRZoneOffset(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.goalCenterOffsetPose);
            waypoints.add(new Pose2d(Constants.goalCenterOffsetPose.transformBy(new Pose2d( new Translation2d(-70.0, 0.0), new Rotation2d(0)))));
            // waypoints.add(rZoneBallOnePose.transformBy(new Pose2d( new Translation2d(5.0, -2.0), new Rotation2d(0))));
            // waypoints.add(trenchBallTwoPose);
            return generateTrajectory(true, waypoints, Arrays.asList(), 25.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 15);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getRZoneOffsetToCloserRZone(){
            List<Pose2d> waypoints = new ArrayList<>();
            // waypoints.add(goalCenterOffsetPose);
            waypoints.add(new Pose2d(Constants.goalCenterOffsetPose.transformBy(new Pose2d( new Translation2d(-70.0, 0.0), new Rotation2d(0)))));
            waypoints.add(Constants.rZoneBallOnePose.transformBy(new Pose2d( new Translation2d(15.0, -6.0), new Rotation2d(0))));
            // waypoints.add(trenchBallTwoPose);
            return generateTrajectory(true, waypoints, Arrays.asList(), 50.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 15);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getCloserRZoneToNearTrench(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.rZoneBallOnePose.transformBy(new Pose2d( new Translation2d(15.0, -6.0), new Rotation2d(-110))));
            waypoints.add(new Pose2d(new Translation2d(440.0, 110.0), new Rotation2d(-110.0)));
            waypoints.add(new Pose2d(new Translation2d(385.0, 130.0), new Rotation2d(-110.0)));
            //waypoints.add(new Pose2d( rZoneBallOne110Pose.transformBy( Pose2d.fromTranslation( new Translation2d(50.0, 40.0)))));
            //waypoints.add(new Pose2d( rZoneBallOne110Pose.transformBy( Pose2d.fromTranslation( new Translation2d(0, 75.0)))));
            // waypoints.add(trenchBallTwoPose);
            return generateTrajectory(true, waypoints, Arrays.asList(), 50.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 15);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getGoalCenterOffsetLeftToFarRZone(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.goalCenterOffsetLeftPose);
            waypoints.add(new Pose2d(new Translation2d(370.0, -50.0), new Rotation2d(0.0)));

            return generateTrajectory(true, waypoints, Arrays.asList(), 80.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRZoneLeftToBallOne(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.outsideRZoneLeftPose);
            waypoints.add(new Pose2d(new Translation2d(400.0, 20.0), new Rotation2d(70.0)));
            // waypoints.add(new Pose2d(new Translation2d(370.0, 20.0), new Rotation2d(70.0)));
            

            return generateTrajectory(false, waypoints, Arrays.asList(), 80.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }
        
        private Trajectory<TimedState<Pose2dWithCurvature>> getRZoneBallOneToInside(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.rZoneBallOneAnglePose);
            waypoints.add(new Pose2d(new Translation2d(360.0, 20.0), new Rotation2d(70.0)));
            // waypoints.add(new Pose2d(new Translation2d(370.0, 20.0), new Rotation2d(70.0)));
            

            return generateTrajectory(false, waypoints, Arrays.asList(), 80.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRZoneInsideToNearTrenchGoal(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.rZoneInsidePose);
            waypoints.add(new Pose2d(new Translation2d(395.0, 80.0), new Rotation2d(70.0)));
            // waypoints.add(new Pose2d(new Translation2d(370.0, 20.0), new Rotation2d(70.0)));
            
            return generateTrajectory(false, waypoints, Arrays.asList(), 80.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLoadWallToEnemyTrench(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.loadWallPose);
            waypoints.add(Constants.testEnemyTrenchBallFourAndFive);
            // waypoints.add(new Pose2d(new Translation2d(370.0, 20.0), new Rotation2d(70.0)));
            

            return generateTrajectory(false, waypoints, Arrays.asList(), 75.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getEnemyTrenchToGoalOffsetLeft(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(Constants.testEnemyTrenchBallFourAndFive.getTranslation(), Rotation2d.fromDegrees(180.0)));
            // waypoints.add(new Pose2d(new Translation2d(450.0, -65.0), new Rotation2d(90.0)));
            waypoints.add(new Pose2d(Constants.goalCenterOffsetLeftPose.getTranslation(), Rotation2d.fromDegrees(270.0)));
            // waypoints.add(new Pose2d(new Translation2d(370.0, 20.0), new Rotation2d(70.0)));
            

            return generateTrajectory(true, waypoints, Arrays.asList(), 100.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getBackPocketOffsetToEnemyRZone(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.goalCenterOffsetPose);
            waypoints.add(new Pose2d(new Translation2d(350.0, 80.0), new Rotation2d(0.0)));
            waypoints.add(Constants.enemyRZoneBallFivePose);
            // waypoints.add(new Pose2d(new Translation2d(370.0, 20.0), new Rotation2d(70.0)));
            

            return generateTrajectory(true, waypoints, Arrays.asList(), 80.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getWallToRZoneTwoBall(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.goalWallPose);
            waypoints.add(Constants.rZoneBallOnePose);
            // waypoints.add(new Pose2d(new Translation2d(370.0, 20.0), new Rotation2d(70.0)));
            
            return generateTrajectory(true, waypoints, Arrays.asList(), 25.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        } 

        private Trajectory<TimedState<Pose2dWithCurvature>> getRZoneTwoBallToTrenchThreeBall(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.rZoneBallOnePose);
            waypoints.add(new Pose2d(new Translation2d(415.0, -105.0), new Rotation2d(0.0)));
            waypoints.add(new Pose2d(new Translation2d(365.0, -128.0), new Rotation2d(0.0)));
            waypoints.add(Constants.trenchBallThreePose);

            return generateTrajectory(true, waypoints, Arrays.asList(), 25.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTrenchThreeBallToTrenchCloseShot(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.trenchBallThreePose);
            waypoints.add(Constants.trenchBallOnePose);

            return generateTrajectory(true, waypoints, Arrays.asList(), 25.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRZoneTwoBallToRZoneThreeBall(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.rZoneBallOnePose);
            waypoints.add(new Pose2d(new Translation2d(450.0, -55.0), new Rotation2d(0.0)));
            waypoints.add(Constants.goalCenterOffsetLeftPose);
            waypoints.add(new Pose2d(new Translation2d(421.0, -6.0), new Rotation2d(0.0)));

            return generateTrajectory(true, waypoints, Arrays.asList(), 25.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRZoneBallRun(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(421.0, -6.0), new Rotation2d(0.0)));
            waypoints.add(new Pose2d(new Translation2d(411.0, 21.0), new Rotation2d(0.0)));

            return generateTrajectory(true, waypoints, Arrays.asList(), 25.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRZoneBallRunToGoalOffsetLeft(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(411.0, 21.0), new Rotation2d(0.0)));
            waypoints.add(Constants.goalCenterOffsetLeftPose);

            return generateTrajectory(true, waypoints, Arrays.asList(), 25.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }


        //180* Paths

        private Trajectory<TimedState<Pose2dWithCurvature>> getWallToOffsetBackward(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.goalWallBackwardPose);
            waypoints.add(Constants.goalCenterOffsetBackwardPose);
            // waypoints.add(new Pose2d(new Translation2d(495.0, -142.0), new Rotation2d(0.0)));
            // waypoints.add(new Pose2d(new Translation2d(495.0, -110.0), new Rotation2d(0.0)));
            return generateTrajectory(false, waypoints, Arrays.asList(), 50.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getWallToTrenchBallThree(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.goalWallPose);
            waypoints.add(Constants.testTrenchBallThree);
            // waypoints.add(new Pose2d(new Translation2d(495.0, -142.0), new Rotation2d(0.0)));
            // waypoints.add(new Pose2d(new Translation2d(495.0, -110.0), new Rotation2d(0.0)));
            return generateTrajectory(false, waypoints, Arrays.asList(), 100.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getOffsetToTrenchBallOneBackward(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.goalCenterOffsetBackwardPose);
            waypoints.add(Constants.trenchBallOneBackwardPose);
            return generateTrajectory(false, waypoints, Arrays.asList(), 0.0, 35.0, 35.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 15);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTrenchBallOneToTrenchFarShotBackward(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.trenchBallOneBackwardPose);
            waypoints.add(Constants.trenchBallTwoBackwardPose);
            waypoints.add(Constants.trenchBallThreeBackwardPose);
            waypoints.add(Constants.trenchBallFourPose.transformBy(new Pose2d(new Translation2d(15.0,0.0), new Rotation2d(0.0))));
            // waypoints.add(trenchBallFourPose.transformBy(new Pose2d(new Translation2d(0.0,-5.0), new Rotation2d(0.0))));
            return generateTrajectory(false, waypoints, Arrays.asList(), 35.0, 0.0, 100.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 30);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTrenchFullToTrenchCloseShotBackward(){
            List<Pose2d> waypoints = new ArrayList<>();
            // waypoints.add(trenchBallFourPose.transformBy(new Pose2d(new Translation2d(0.0,-5.0), new Rotation2d(0.0))));
            waypoints.add(Constants.trenchBallFourBackwardPose);
            waypoints.add(Constants.trenchBallThreeBackwardPose);
            waypoints.add(Constants.trenchBallTwoBackwardPose);
            waypoints.add(Constants.trenchBallOneBackwardPose);

            return generateTrajectory(true, waypoints, Arrays.asList(), 100.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 15);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLoadWallToEnemyTrenchBackward() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.loadWallBackwardPose);
            waypoints.add(Constants.enemyTrenchBackwardPose);

            return generateTrajectory(false, waypoints, Arrays.asList(), 25.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getEnemyTrenchToGoalOffsetLeftBackward(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(Constants.enemyTrenchBackwardPose.getTranslation(), Rotation2d.fromDegrees(10.0)));
            // waypoints.add(new Pose2d(new Translation2d(205.0, 61.0), new Rotation2d(0.0)));
            // waypoints.add(new Pose2d(new Translation2d(175.0, -10.0), new Rotation2d(0.0)));
            waypoints.add(new Pose2d(Constants.goalCenterOffsetLeftBackwardPose.getTranslation(), Rotation2d.fromDegrees(90)));
            // waypoints.add(new Pose2d(new Translation2d(370.0, 20.0), new Rotation2d(70.0)));
            
            return generateTrajectory(true, waypoints, Arrays.asList(), 25.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getWallToRZoneTwoBallBackward(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.goalWallBackwardPose);
            waypoints.add(new Pose2d(new Translation2d(200.0, -110.0), new Rotation2d(0.0)));
            waypoints.add(Constants.rZoneBallOneBackwardPose);
            // waypoints.add(new Pose2d(new Translation2d(370.0, 20.0), new Rotation2d(70.0)));
            
            return generateTrajectory(false, waypoints, Arrays.asList(), 75.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRZoneTwoBallToNearTrenchBackward(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.rZoneBallOneBackwardPose.transformBy(new Pose2d(new Translation2d(0.0,0.0), new Rotation2d(70.0))));
            waypoints.add(new Pose2d(new Translation2d(215.0, -100.0), new Rotation2d(70.0)));
            waypoints.add(new Pose2d(new Translation2d(190.0, -138.0), new Rotation2d(70.0)));
            // waypoints.add(trenchBallThreeBackwardPose.transformBy(new Pose2d(new Translation2d(0.0,0.0), new Rotation2d(0.0))));

            return generateTrajectory(true, waypoints, Arrays.asList(), 60.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getNearTrenchToTrenchBallThreeBackward(){
            List<Pose2d> waypoints = new ArrayList<>();
            // waypoints.add(rZoneBallOneBackwardPose.transformBy(new Pose2d(new Translation2d(0.0,0.0), new Rotation2d(70.0))));
            // waypoints.add(new Pose2d(new Translation2d(215.0, -100.0), new Rotation2d(70.0)));
            waypoints.add(new Pose2d(new Translation2d(190.0, -138.0), new Rotation2d(0.0)));
            waypoints.add(Constants.trenchBallThreeBackwardPose.transformBy(new Pose2d(new Translation2d(-10.0,-10.0), new Rotation2d(0.0))));

            return generateTrajectory(false, waypoints, Arrays.asList(), 80.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTrenchThreeBallToTrenchCloseShotBackward(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.trenchBallThreeBackwardPose.transformBy(new Pose2d(new Translation2d(-10.0,-10.0), new Rotation2d(0.0))));
            waypoints.add(Constants.trenchBallOneBackwardPose.transformBy(new Pose2d(new Translation2d(-20.0,-10.0), new Rotation2d(0.0))));

            return generateTrajectory(true, waypoints, Arrays.asList(), 80.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRZoneTwoBallToRZoneOffsetBackward(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.rZoneBallOneBackwardPose.transformBy(new Pose2d(new Translation2d(0.0,0.0), new Rotation2d(0.0))));
            waypoints.add(new Pose2d(new Translation2d(200.0, -85.0), new Rotation2d(0.0)));
            waypoints.add(new Pose2d(new Translation2d(165.0, -85.0), new Rotation2d(0.0)));
            // waypoints.add(goalCenterOffsetLeftBackwardPose);
            // waypoints.add(new Pose2d(new Translation2d(204.0, -6.0), new Rotation2d(0.0)));
            // waypoints.add(new Pose2d(new Translation2d(370.0, 20.0), new Rotation2d(70.0)));
            
            return generateTrajectory(true, waypoints, Arrays.asList(), 75.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRZoneOffsetToRZoneThreeBallBackward(){
            List<Pose2d> waypoints = new ArrayList<>();
            // waypoints.add(rZoneBallOneBackwardPose.transformBy(new Pose2d(new Translation2d(0.0,0.0), new Rotation2d(70.0))));
            waypoints.add(new Pose2d(new Translation2d(165.0, -85.0), new Rotation2d(0.0)));
            waypoints.add(new Pose2d(new Translation2d(165.0, -10.0), new Rotation2d(0.0)));
            waypoints.add(new Pose2d(new Translation2d(204.0, -8.0), new Rotation2d(0.0)));
            // waypoints.add(new Pose2d(new Translation2d(204.0, -6.0), new Rotation2d(0.0)));
            // waypoints.add(new Pose2d(new Translation2d(370.0, 20.0), new Rotation2d(70.0)));
            
            return generateTrajectory(true, waypoints, Arrays.asList(), 75.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRZoneBallRunBackward(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(204.0, -8.0), new Rotation2d(0.0)));
            waypoints.add(new Pose2d(new Translation2d(220.0, 23.0), new Rotation2d(0.0)));

            return generateTrajectory(true, waypoints, Arrays.asList(), 50.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRZoneBallRunToGoalOffsetLeftBackward(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(222.0, 23.0), new Rotation2d(0.0)));
            waypoints.add(new Pose2d(new Translation2d(170.0, 23.0), new Rotation2d(0.0)));
            waypoints.add(new Pose2d(new Translation2d(170.0, -40.0), new Rotation2d(0.0)));

            return generateTrajectory(true, waypoints, Arrays.asList(), 100.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getSquigly(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(120.0, 145.0), new Rotation2d(0.0)));
            waypoints.add(new Pose2d(new Translation2d(120.0, 45.0), new Rotation2d(180.0)));
            waypoints.add(new Pose2d(new Translation2d(120.0, -45.0), new Rotation2d(0.0)));
            waypoints.add(new Pose2d(new Translation2d(120.0, -145.0), new Rotation2d(180.0)));

            return generateTrajectory(true, waypoints, Arrays.asList(), 30.0, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }
    }
}
