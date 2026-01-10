package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

    public CTREConfigs() {
        /** Swerve CANCoder Configuration */
        swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants.DriveConstants.cancoderInvert;

        /** Swerve Angle Motor Configurations */
        /* Motor Inverts and Neutral Mode */
        swerveAngleFXConfig.MotorOutput.Inverted = Constants.DriveConstants.angleMotorInvert;
        swerveAngleFXConfig.MotorOutput.NeutralMode = Constants.DriveConstants.angleNeutralMode;

        /* Gear Ratio and Wrapping Config */
        swerveAngleFXConfig.Feedback.SensorToMechanismRatio = Constants.DriveConstants.angleGearRatio;
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;

        /* Current Limiting */
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.DriveConstants.angleEnableCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.DriveConstants.angleCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLowerLimit = Constants.DriveConstants.angleCurrentThreshold;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLowerTime = Constants.DriveConstants.angleCurrentThresholdTime;

        /* PID Config */
        swerveAngleFXConfig.Slot0.kP = Constants.DriveConstants.angleKP;
        swerveAngleFXConfig.Slot0.kI = Constants.DriveConstants.angleKI;
        swerveAngleFXConfig.Slot0.kD = Constants.DriveConstants.angleKD;

        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        swerveDriveFXConfig.MotorOutput.Inverted = Constants.DriveConstants.driveMotorInvert;
        swerveDriveFXConfig.MotorOutput.NeutralMode = Constants.DriveConstants.driveNeutralMode;

        /* Gear Ratio Config */
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = Constants.DriveConstants.driveGearRatio;

        /* Current Limiting */
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.DriveConstants.driveEnableCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.DriveConstants.driveCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLowerLimit = Constants.DriveConstants.driveCurrentThreshold;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLowerTime = Constants.DriveConstants.driveCurrentThresholdTime;

        /* PID Config */
        swerveDriveFXConfig.Slot0.kP = Constants.DriveConstants.driveKP;
        swerveDriveFXConfig.Slot0.kI = Constants.DriveConstants.driveKI;
        swerveDriveFXConfig.Slot0.kD = Constants.DriveConstants.driveKD;

        /* Open and Closed Loop Ramping */
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.DriveConstants.openLoopRamp;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.DriveConstants.openLoopRamp;

        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.DriveConstants.closedLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.DriveConstants.closedLoopRamp;
    }
}