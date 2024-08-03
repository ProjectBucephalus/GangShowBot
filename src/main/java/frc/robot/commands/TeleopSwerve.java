package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.GANG_SHOW_CONSTANTS;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.ControllConstants.stickDeadband)*GANG_SHOW_CONSTANTS.maxSpin;
        double translationVal = translationSup.getAsDouble();
        double strafeVal = strafeSup.getAsDouble();
        if (Math.sqrt(Math.pow(translationSup.getAsDouble(), 2) + Math.pow(strafeSup.getAsDouble(), 2)) <= Constants.ControllConstants.stickDeadband) 
        {
            translationVal = 0;
            strafeVal = 0;   
        }
        translationVal = translationVal * GANG_SHOW_CONSTANTS.maxSpeed;
        strafeVal = strafeVal * GANG_SHOW_CONSTANTS.maxSpeed;

        /* if (Swerve.withinBounds(translationVal, strafeVal))
        {
            
            s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
                rotationVal * Constants.Swerve.maxAngularVelocity, 
                !robotCentricSup.getAsBoolean(), 
                true
            );
        } */

        s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
                rotationVal * Constants.Swerve.maxAngularVelocity, 
                !robotCentricSup.getAsBoolean(), 
                true
            );
    }
}