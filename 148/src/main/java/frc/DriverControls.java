// package frc;

// import java.util.Arrays;

// import frc.loops.LimelightProcessor;
// import frc.loops.Loop;
// import frc.subsystems.Hanger;
// import frc.subsystems.LEDs;
// import frc.subsystems.Shooter;
// import frc.subsystems.SubsystemManager;
// import frc.subsystems.Superstructure;
// import frc.subsystems.Swerve;
// import frc.subsystems.Turret;
// import com.team1323.io.Xbox;
// import com.team1323.lib.util.Util;
// import com.team254.lib.geometry.Pose2d;
// import com.team254.lib.geometry.Rotation2d;
// import com.team254.lib.geometry.Translation2d;

// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.GenericHID.Hand;

// /**
//  * A class to assign controller inputs to robot actions
//  */
// public class DriverControls implements Loop {

//     private static DriverControls instance = null;

//     public static DriverControls getInstance() {
//         if (instance == null)
//             instance = new DriverControls();
//         return instance;
//     }

// 	Xbox driver, coDriver;

//     private Swerve swerve;
//     private Turret turret;
//     private Shooter shooter;
//     private Hanger hanger;
//     private Superstructure s;

//     private SubsystemManager subsystems;
//     public SubsystemManager getSubsystems(){ return subsystems; }

//     private RobotState robotState;

//     private LimelightProcessor limelight;

//     private final boolean oneControllerMode = false;
// 	private boolean flickRotation = false;
//     private boolean robotCentric = false;
//     private boolean isTurret180Rotation = false;
        
//     private boolean inAuto = true;
//     public void setAutoMode(boolean auto) {
//         inAuto = auto;
//     }

//     public boolean getInAuto() {
//         return inAuto;
//     }

//     public DriverControls() {
//         driver = new Xbox(0);
// 		coDriver = new Xbox(1);
//         driver.setDeadband(0.0);
// 		coDriver.setDeadband(0.25);
// 		coDriver.rightBumper.setLongPressDuration(1.0);

//         swerve = Swerve.getInstance();
//         turret = Turret.getInstance();
//         shooter = Shooter.getInstance();
//         hanger = Hanger.getInstance();
//         s = Superstructure.getInstance();

//         subsystems = new SubsystemManager(
// 				Arrays.asList(swerve, turret, shooter, hanger, s));

//         robotState = RobotState.getInstance();

//         limelight = LimelightProcessor.getInstance();
//     }

//     @Override
//     public void onStart(double timestamp) {
//         if(inAuto) {
//             swerve.zeroSensors();
//             swerve.setNominalDriveOutput(0.0);
//             swerve.requireModuleConfiguration();
//         } else {
//             swerve.setNominalDriveOutput(0.0);
//             swerve.set10VoltRotationMode(false);
//         }
//     }

//     @Override
//     public void onLoop(double timestamp) {
//         if(inAuto) {
//             if(s.swerve.getState() == Swerve.ControlState.VISION_PID){
//             }else{
//             }
//         } else {
//             driver.update();
// 			coDriver.update();
//             if(oneControllerMode) oneControllerMode();
//             else twoControllerMode();
//             // String message = DriverStation.getInstance().getGameSpecificMessage();
//             // if (message != null)
//             //     switch(message) {
//             //     case "R":
//             //     wheelOfFortune.setColorTarget(SliceColor.RED);
//             //     break;
//             //     case "G":
//             //     wheelOfFortune.setColorTarget(SliceColor.GREEN);
//             //     break;
//             //     case "B":
//             //     wheelOfFortune.setColorTarget(SliceColor.BLUE);
//             //     break;
//             //     case "Y":
//             //     wheelOfFortune.setColorTarget(SliceColor.YELLOW);
//             //     break;
//             //     default:
//             //     break;
//             //     }
//         }
//     }

//     @Override
//     public void onStop(double timestamp) {
//         subsystems.stop();
//     }

//     private void twoControllerMode() {
//         double swerveYInput = driver.getX(Hand.kLeft);
//         double swerveXInput = -driver.getY(Hand.kLeft);
//         double swerveRotationInput = (flickRotation ? 0.0 : driver.getX(Hand.kRight));
        
//         swerve.sendInput(swerveXInput, swerveYInput, swerveRotationInput, robotCentric, driver.leftTrigger.isBeingPressed());
        
//         if (driver.rightCenterClick.shortReleased()) {
//             /*if (flickRotation) {
//                 driver.rumble(3, 1);
//             } else {
//                 driver.rumble(1, 1);
//             }
//             flickRotation = !flickRotation;*/
//         }
        
//         if (flickRotation) {
//             swerve.updateControllerDirection(new Translation2d(-driver.getY(Hand.kRight), driver.getX(Hand.kRight)));
//             if (!Util.epsilonEquals(
//             Util.placeInAppropriate0To360Scope(swerve.getTargetHeading(),
//             swerve.averagedDirection.getDegrees()),
//             swerve.getTargetHeading(), swerve.rotationDivision / 2.0)) {
//                 swerve.rotate(swerve.averagedDirection.getDegrees());
//             }
//         }
        
//         if (driver.bButton.isBeingPressed())
//         swerve.rotate(90);
//         else if (driver.aButton.isBeingPressed()) 
//         swerve.rotate(180);
//         else if (driver.xButton.isBeingPressed())
//         swerve.rotate(270);
//         else if (driver.yButton.isBeingPressed())
//         swerve.rotate(0.0);

//         if (driver.startButton.isBeingPressed()) 
//             swerve.setState(Swerve.ControlState.NEUTRAL);

//         if (driver.backButton.shortReleased() || driver.backButton.longPressed()) {
//             swerve.temporarilyDisableHeadingController();
//             swerve.zeroSensors(new Pose2d());
//             swerve.resetAveragedDirection();
//         }

// 		////// Official Controls //////
//         double coDriverRightX = coDriver.getX(Hand.kRight);
//         double coDriverLeftY = -coDriver.getY(Hand.kLeft);

//         // if (driver.POV0.wasActivated()) {
//         //     s.deployHangerState();
//         // }

//         // if (hanger.isExtended()) {
//         //     if (coDriver.POV0.wasActivated()) {
//         //         s.hangerState(Constants.Hanger.kMaxControlHeight);
//         //     } else if (coDriver.POV180.wasActivated()) {
//         //         s.hangerState(0.0);
//         //     }

//         //     if (coDriver.rightBumper.wasActivated()) {
//         //         traverse.setOpenLoop(1.0);
//         //     } else if (coDriver.leftBumper.wasActivated()) {
//         //         traverse.setOpenLoop(-1.0);
//         //     } else if (coDriver.leftBumper.wasReleased() || coDriver.rightBumper.wasReleased()) {
//         //         traverse.setOpenLoop(0.0);
//         //     }

//         //     if (driver.leftBumper.wasActivated()) {
//         //         traverse.setOpenLoop(-1.0);
//         //     } else if (driver.rightBumper.wasActivated()) {
//         //         traverse.setOpenLoop(1.0);
//         //     } else if (driver.leftBumper.wasReleased() || driver.rightBumper.wasReleased()) {
//         //         traverse.setOpenLoop(0.0);
//         //     }

//         //     if (Math.abs(coDriverLeftY) != 0) {
//         //         hanger.setOpenLoop(coDriverLeftY);
//         //     } else if (hanger.isOpenLoop()) {
//         //         hanger.lockHeight();
//         //     }
//         // } else {

//             if (Math.abs(coDriverRightX) != 0) {
//                 turret.setOpenLoop(coDriverRightX);
//             } else if (turret.isOpenLoop()) {
//                 turret.lockAngle();
//             }

//             if (coDriver.rightCenterClick.shortReleased() || coDriver.rightCenterClick.longReleased()) {
//                 if (!turret.isGoingToPole()) {
//                     if (Math.abs(turret.getRotation().distance(Rotation2d.fromDegrees(180.0))) < Math.abs(turret.getRotation().distance(Rotation2d.fromDegrees(0.0)))) {
//                         turret.setPosition(180.0);
//                         isTurret180Rotation = true;
//                     } else {
//                         turret.setPosition(0.0);
//                         isTurret180Rotation = false;
//                     }
//                 } else {
//                     if (isTurret180Rotation) {
//                         turret.setPosition(0.0);
//                         isTurret180Rotation = false;
//                     } else {
//                         turret.setPosition(180.0);
//                         isTurret180Rotation = true;
//                     }
//                 }
//             }
    
//             // if (coDriver.rightBumper.shortReleased() || coDriver.rightBumper.longReleased()) 
//             //     s.intakeState();
    
//             // if (coDriver.startButton.wasActivated())  {
//             //     intake.conformToState(Intake.State.HUMAN_LOADING);
//             //     feeder.conformToState(Feeder.State.RECEIVING);
//             // }
    
//             // if (coDriver.leftBumper.wasActivated()) {
//             //     intake.conformToState(Intake.State.EJECTING);
//             // } else if (coDriver.leftBumper.wasReleased()) {
//             //     intake.conformToState(Intake.State.OFF);
//             // }
    
//             // if (coDriver.leftCenterClick.wasActivated()) 
//             //     feeder.conformToState(Feeder.State.REVERSE);
//             // else if (coDriver.leftCenterClick.wasReleased())  
//             //     feeder.conformToState(Feeder.State.OFF);
    
//             if (driver.rightTrigger.wasActivated()) {
//                 s.manualCloseShot();
//                 //s.testMidShot();
//                 //s.testFarShot();
//             } else if (driver.rightTrigger.wasReleased()) {
//                 driver.rumble(1.0, 2.0);
//                 coDriver.rumble(1.0, 2.0);
//                 s.postManualShooting();
//             }
    
//             // if (coDriver.leftTrigger.wasActivated()) {
//             //     s.preFireState();
//             // } 

//             if (coDriver.rightTrigger.isBeingPressed()) {
//                 if (coDriver.aButton.wasActivated()) {
//                     s.closeShot();
//                 } else if (coDriver.xButton.wasActivated()) {
//                     s.midShot();
//                 } else if (coDriver.yButton.wasActivated()) {
//                     s.farShot();
//                 }
//             } else if (!coDriver.rightTrigger.isBeingPressed()) {
//                 if (coDriver.aButton.longPressed()) {
//                     s.closeProtectedMoveAndFireState();
//                 } else if (coDriver.xButton.longPressed()) {
//                     s.midVisionMoveAndFireState();
//                 } else if (coDriver.yButton.longPressed()) {
//                     s.farVisionMoveAndFireState();
//                 }
//             }
    
//             if (coDriver.POV0.wasActivated()) {
//                 s.turretPositionState(0.0);
//             } else if (coDriver.POV90.wasActivated()) {
//                 s.turretPositionState(90.0);
//             } else if (coDriver.POV180.wasActivated()) {
//                 s.turretPositionState(180.0);
//             }



//         if (coDriver.backButton.isBeingPressed()){
//             s.neutralState();
//             swerve.setState(Swerve.ControlState.NEUTRAL);
//         }

//     }

//     private void oneControllerMode() {

//     }
// }
