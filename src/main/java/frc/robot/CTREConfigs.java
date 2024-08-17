package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public final class CTREConfigs {
    //public TalonFXConfiguration swerveAngleFXConfigAlt = new TalonFXConfiguration(); // ERROR: The physical gearing on the specific robot is built wrong, remove this if all swerve modules are built correctly
    public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

    public CTREConfigs(){
        /** Swerve CANCoder Configuration */
        swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants.Swerve.cancoderInvert;

        /** Swerve Angle Motor Configurations */
        /* Motor Inverts and Neutral Mode */
        swerveAngleFXConfig.MotorOutput.Inverted = Constants.Swerve.angleMotorInvert;
        swerveAngleFXConfig.MotorOutput.NeutralMode = Constants.Swerve.angleNeutralMode;

        /* Gear Ratio and Wrapping Config */
        swerveAngleFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.angleGearRatio;
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;
        
        /* Current Limiting */
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.angleEnableCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.angleCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.Swerve.angleCurrentThreshold;
        swerveAngleFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.Swerve.angleCurrentThresholdTime;

        /* PID Config */
        swerveAngleFXConfig.Slot0.kP = Constants.Swerve.angleKP;
        swerveAngleFXConfig.Slot0.kI = Constants.Swerve.angleKI;
        swerveAngleFXConfig.Slot0.kD = Constants.Swerve.angleKD;

    // ERROR: The physical gearing of the specific robot is built wrong, remove the following chunk if all swerve modules are built correctly!
    //    /* Motor Inverts and Neutral Mode */
    //    swerveAngleFXConfigAlt.MotorOutput.Inverted = Constants.Swerve.angleMotorInvert;
    //    swerveAngleFXConfigAlt.MotorOutput.NeutralMode = Constants.Swerve.angleNeutralMode;
//
    //    /* Gear Ratio and Wrapping Config */
    //    swerveAngleFXConfigAlt.Feedback.SensorToMechanismRatio = Constants.Swerve.angleGearRatioAlt;
    //    swerveAngleFXConfigAlt.ClosedLoopGeneral.ContinuousWrap = true;
    //    
    //    /* Current Limiting */
    //    swerveAngleFXConfigAlt.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.angleEnableCurrentLimit;
    //    swerveAngleFXConfigAlt.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.angleCurrentLimit;
    //    swerveAngleFXConfigAlt.CurrentLimits.SupplyCurrentThreshold = Constants.Swerve.angleCurrentThreshold;
    //    swerveAngleFXConfigAlt.CurrentLimits.SupplyTimeThreshold = Constants.Swerve.angleCurrentThresholdTime;
//
    //    /* PID Config */
    //    swerveAngleFXConfigAlt.Slot0.kP = Constants.Swerve.angleKP;
    //    swerveAngleFXConfigAlt.Slot0.kI = Constants.Swerve.angleKI;
    //    swerveAngleFXConfigAlt.Slot0.kD = Constants.Swerve.angleKD;
    // ERROR: The physical gearing on the specific robot is built wrong, remove the above chunk if all swerve modules are built correctly

        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        swerveDriveFXConfig.MotorOutput.Inverted = Constants.Swerve.driveMotorInvert;
        swerveDriveFXConfig.MotorOutput.NeutralMode = Constants.Swerve.driveNeutralMode;

        /* Gear Ratio Config */
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.driveGearRatio;

        /* Current Limiting */
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.driveEnableCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.driveCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.Swerve.driveCurrentThreshold;
        swerveDriveFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.Swerve.driveCurrentThresholdTime;

        /* PID Config */
        swerveDriveFXConfig.Slot0.kP = Constants.Swerve.driveKP;
        swerveDriveFXConfig.Slot0.kI = Constants.Swerve.driveKI;
        swerveDriveFXConfig.Slot0.kD = Constants.Swerve.driveKD;

        /* Open and Closed Loop Ramping */
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;

        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;

    }
}