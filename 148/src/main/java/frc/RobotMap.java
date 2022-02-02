package frc;

//CAN IDS on the robot

public class RobotMap {
    //Motor Controllers
    //Map Swerve CAN IDs
    public static final int FRONT_RIGHT_ROTATION= 4;
    public static final int FRONT_RIGHT_DRIVE   = 3;
    public static final int FRONT_LEFT_ROTATION = 2;
    public static final int FRONT_LEFT_DRIVE    = 1;
    public static final int REAR_RIGHT_ROTATION = 8;
    public static final int REAR_RIGHT_DRIVE    = 7;
    public static final int REAR_LEFT_ROTATION  = 6;
    public static final int REAR_LEFT_DRIVE     = 5;

    //Map Swerve Encoder IDs
    public static final int FRONT_RIGHT_CODER = 0;
    public static final int FRONT_LEFT_CODER = 1;
    public static final int REAR_LEFT_CODER = 2;
    public static final int REAR_RIGHT_CODER = 3;

    //Map Turret CAN IDs
    public static final int TURRET_MASTER = 9;

    //Map Flywheel CAN IDs
    public static final int FLYWHEEL_MASTER = 10;
    public static final int FLYWHEEL_FOLLOWER = 11;

    //Map Feeder CAN IDs
    public static final int FEEDER_MASTER = 12;

    //Map Hopper CAN IDs
    public static final int HOPPER_MASTER = 13;

    //Map Intake CAN IDs
    public static final int INTAKE_MASTER = 14;

    //Map IntakePivot CAN IDs
    public static final int INTAKE_PIVOT_MASTER = 15;

    //Map Hanger CAN IDs
    public static final int HANGER_MASTER = 16;

    //Pigeon
    public static final int PIGEON = 17;    
    
    //Digital Inputs

    //CANifier
    public static final int CANIFIER = 23;


    //Hood Servos
    public static final int SERVO_1 = 8;
    public static final int SERVO_2 = 9;

    //Hood Encoder
    // public static final int MA3_PORT = 0;
    public static final int HOOD_CAN_PORT = 6; //DIO 4
}
