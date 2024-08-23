package frc.robot;

/** Values for setting up Gang-Show robot opperation */
public class GANG_SHOW_CONSTANTS 
{

    // ----------------------- //
    // > How the robot moves < //
    // ----------------------- //

    /** Maximum robot speed in metres per second */
    public static final double maxSpeed = 0.5;

    /** Maximum robot rotation speed in radians per second (6 radians is about 1 rotation) */
    public static final double maxSpin = 0.25;

    /** How fast to twist the wheels to a new direction, increase if changing direction is sluggish */
    public static final double wheelTwistRate = 15;


    // -------------------------- //
    // > Where the robot can go < //
    // -------------------------- //

    // Relative to the centre of the robot, in direction the robot is facing
    // These values are the distance in metres to the virtual wall the robot will stop at
    // 0 means the wall is running through the middle of the robot
    // negative distances will have the robot start outside the area, and can only move into it
    /** Metres the robot can travel left */
    public static final double stageLeft = 0.6;

    /** Metres the robot can travel right */
    public static final double stageRight = 10;

    /** Metres the robot can travel forwards */
    public static final double stageFront = 1.65;

    /** Metres the robot can travel back */
    public static final double stageBack = 2.25;    

    // This value is how close in metres the robot can get to the virtual wall before it starts to slow down when moving towards it
    /** Metres buffer distance to the wall */
    public static final double wallBuffer = 1;

    // This value is how close in metres the centre of the robot can get to the wall before it stops
    /** Metres maximum distance from robot centre to robot perimeter */
    public static final double robotRadius = 0.4;

    // This value is how far the virtual position moves when using the D-Pad
    /** Metres odometry shift distance */
    public static final double  odoShift = 0.25;

}
