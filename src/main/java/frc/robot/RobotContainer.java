package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

import frc.robot.GANG_SHOW_CONSTANTS;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final CommandXboxController driver = new CommandXboxController(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    private final int brakeAxis = XboxController.Axis.kRightTrigger.value;

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> driver.getRawAxis(brakeAxis),
                () -> driver.rightBumper().getAsBoolean(),
                () -> false,
                () -> driver.leftTrigger().getAsBoolean()
            )
        );

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() 
    {
        /* Driver Buttons */
        driver.start().onTrue(new InstantCommand(() -> s_Swerve.zeroHeading(0)));
        driver.back().onTrue(new InstantCommand(() -> s_Swerve.zeroPose(0)));
        driver.povUp().onTrue(new InstantCommand(() -> s_Swerve.shiftPose(GANG_SHOW_CONSTANTS.odoShift, 0)));
        driver.povDown().onTrue(new InstantCommand(() -> s_Swerve.shiftPose(-GANG_SHOW_CONSTANTS.odoShift, 0)));
        driver.povLeft().onTrue(new InstantCommand(() -> s_Swerve.shiftPose(0, GANG_SHOW_CONSTANTS.odoShift)));
        driver.povRight().onTrue(new InstantCommand(() -> s_Swerve.shiftPose(0, -GANG_SHOW_CONSTANTS.odoShift)));

        //driver.a().onTrue(new InstantCommand(() -> s_Swerve.setPoseXY(GANG_SHOW_CONSTANTS.xCueA, GANG_SHOW_CONSTANTS.yCueA)));
        //driver.b().onTrue(new InstantCommand(() -> s_Swerve.setPoseXY(GANG_SHOW_CONSTANTS.xCueB, GANG_SHOW_CONSTANTS.yCueB)));
        //driver.x().onTrue(new InstantCommand(() -> s_Swerve.setPoseXY(GANG_SHOW_CONSTANTS.xCueX, GANG_SHOW_CONSTANTS.yCueX)));
        //driver.y().onTrue(new InstantCommand(() -> s_Swerve.setPoseXY(GANG_SHOW_CONSTANTS.xCueY, GANG_SHOW_CONSTANTS.yCueY)));
    }
}
