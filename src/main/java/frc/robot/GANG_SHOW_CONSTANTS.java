package frc.robot;

/** Values for setting up Gang-Show robot opperation */
public class GANG_SHOW_CONSTANTS 
{

    /** Maximum robot speed in metres per second */
    public static final double maxSpeed = 0.5; 
    /** Maximum robot rotation speed in radians per second (6 radians is about 1 rotation) */
    public static final double maxAngularVelocity = 3;
    /** How fast to twist the wheels to a new direction, increase if changing direction is sluggish */
    public static final double wheelTwistRate = 12;

    // >>> Geo-fence setup is not implimented yet <<< //
    /** Metres the robot can travel left */
    public static final double stageLeft = -1;
    /** Metres the robot can travel right */
    public static final double stageRight = 10; 
    /** Metres the robot can travel forwards */
    public static final double stageFront = 4;     
    /** Metres the robot can travel back */
    public static final double stageBack = -1;    

}
