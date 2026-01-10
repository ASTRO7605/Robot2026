package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.Arrays;

import org.photonvision.PhotonUtils;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Time;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {

    public static final boolean kEnableVision = true;

    public static final class DriveConstants {
        public static final int kXboxControllerID = 0;
        public static final int kTurnStickID = 1;
        public static final int kThrottleStickID = 2;

        public static final double kControllerMovementDeadband = 0.1;
        public static final double kControllerRotationDeadband = 0.1;

        public static final double kTimeBeforeBrakeDisabled = 1;

        public static final int pigeonID = 1;
        public static final String kCanivoreBusId = "canivore";

        public static final COTSTalonFXSwerveConstants chosenModule = COTSTalonFXSwerveConstants.SDS.MK4i
                .KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double trackWidth = 0.57785; // meters
        public static final double wheelBase = 0.66675; // meters
        public static final double wheelCircumference = chosenModule.wheelCircumference;
        public static final double kRobotWidth = 0.884; // meters
        public static final double kRobotLength = 0.973; // meters

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 40;
        public static final int angleCurrentThreshold = 50;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 60;
        public static final int driveCurrentThreshold = 70;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.25;

        /* Angle Motor PID Values */
        public static final double angleKP = 100;
        public static final double angleKI = 0;
        public static final double angleKD = 0;

        /* Drive Motor PID Values */
        public static final double driveKP = 1;
        public static final double driveKI = 0;
        public static final double driveKD = 0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.12498;
        public static final double driveKV = 2.4;
        public static final double driveKA = 0.0353;

        public static final double kXYMovementP = 5;
        public static final double kXYMovementI = 0;
        public static final double kXYMovementD = 0;

        public static final double kRotationMovementP = 5;
        public static final double kRotationMovementI = 0;
        public static final double kRotationMovementD = 0;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double kMaxTeleopSpeed = 4.8
                ;
        /** Radians per Second */
        public static final double kMaxTeleopRotateSpeed = 5 * Math.PI;

        public static final double kGeneralSpeedMulti = .2;
        public static final double kDriverMultiSlow = 0.15;

        /* Reef Approach Constants */
        /** Meters per Second */
        public static final double kReefStartApproachDesiredSpeed = 0.75;

        /** Meters per Second */
        public static final double kReefApproachMaxSpeed = 0.75;
        /** Meters per Second ^ 2 */
        public static final double kReefApproachMaxAccel = 1.5;
        /** Radians per Second */
        public static final double kReefApproachMaxRotateSpeed = 2 * Math.PI;
        /** Radians per Second ^ 2 */
        public static final double kReefApproachMaxRotateAccel = 2 * Math.PI;

        /** Meters per Second */
        public static final double kReefPathfindMaxSpeed = 1.25;
        /** Meters per Second ^ 2 */
        public static final double kReefPathfindMaxAccel = 2.25;
        /** Radians per Second */
        public static final double kReefPathfindMaxRotateSpeed = 2 * Math.PI;
        /** Radians per Second ^ 2 */
        public static final double kReefPathfindMaxRotateAccel = 2 * Math.PI;

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(29.75); // Anciennement
            // -152.95
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                    angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-59.2); // Anciennement
            // 138.65
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                    angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-52.95); // Anciennement
            // -53.65
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                    angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-132.25); // Anciennement
            // -149.55
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                    angleMotorID,
                    canCoderID, angleOffset);
        }
    }

    public static final class AlgaeToolConstants {
        public static final int kMotorId = 14;
        public static final int kPositionThreshold = 1;

        public static final double kPFlywheel = 0.0125;
        public static final double kIFlywheel = 0;
        public static final double kDFlywheel = 0;
        public static final double kFFFlywheel = 0;

        public static final double kMaxVelocity = 1;
        public static final double kMaxAcceleration = 1;
        public static final double kAllowedError = 1;

        public static final double kMinPosWhenRestricted = 55;

        public static final double kVoltageCompensation = 11;
        public static final int kCurrentLimit = 30;

        public static final double kManualVoltage = 0.5;
        public static final double kManualSpeed = 0.1;

        public static final double kSoftLimitForward = 170;
        public static final double kSoftLimitReverse = 30;

        public enum AlgaeToolPosition {
            Stow(155),
            Low(50);

            AlgaeToolPosition(double value) {
                this.value = value;
            }

            public final double value;
        }

    }

    public static final class CoralToolConstants {
        public static final int kMotorId = 16;
        public static final double kPositionThreshold = 1.5;

        public static final double kToolDistanceFromCenterMeters = 0.28;

        public static final double kP = 0.015;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kMaxFF = 0.32;
        // since using cos, make sure that pos - offset = 0 when using 100% of maxFF
        public static final double kFFOffset = 0;

        public static final double kGeneralMaxVelocity = 400;
        public static final double kGeneralMaxAcceleration = 1250;

        public static final double kVoltageCompensation = 11;
        public static final int kCurrentLimit = 50;

        public static final double kAutomaticInitVoltage = 0.5;
        public static final double kThresholdMotorStopped = 1;
        public static final double kMinAutoInitTime = 0.1;

        // Degrees
        public static final double fPositionConversion = 360.0 / 50.0;
        // Degrees / second
        public static final double fVelocityConversion = fPositionConversion / 60.0;

        // degrees
        public static final double kLimitSwitchPosition = 180;
        public static final double kHardStopOffset = 8;
        public static final double kReverseSoftLimit = -55;

        public static final double kMinPosWhenRestricted = -10;

        public static final int kSwitchChannel = 1;

        public enum CoralToolPosition {
            Stow(90),
            L1Deposit(-40),
            L2And3Deposit(120),
            L4Deposit(135),
            Loading(-40);

            CoralToolPosition(double value) {
                this.value = value;
            }

            public final double value;
        }

        public static final double kManualSpeed = 0.1;
    }

    public static final class ElevatorConstants {
        public static final int kLeftMotorId = 19;
        public static final int kRightMotorId = 20;
        public static final int kServoId = 21;
        public static final double kPositionThreshold = 0.25;

        public static final double kPRegular = 0.25;
        public static final double kIRegular = 0;
        public static final double kDRegular = 0;

        public static final double kPClimb = 0.25;
        public static final double kIClimb = 0.75;
        public static final double kDClimb = 0;

        public static final double kGeneralMaxVelocity = 40;
        public static final double kGeneralMaxAcceleration = 100;

        public static final double kClimbMaxVelocity = 5;
        public static final double kClimbMaxAcceleration = 10;

        public static final double kFeedForward = 0.7;

        public static final double fPositionConversion = 3 * Math.PI / 10;
        public static final double fVelocityConversion = fPositionConversion / 60;

        public static final double kVoltageCompensation = 11;

        public static final int kRegularCurrentLimit = 50;
        public static final int kClimbCurrentLimit = 80;

        public static final double kMotorRatio = 10;

        public static final double kLimitSwitchPosition = 0;
        public static final double kSoftLimitForward = 55.5;

        public static final double kManualSpeed = 0.1;
        public static final double kInitSpeed = -0.1;

        public static final double kClimbFreeSpeed = -0.1;
        public static final double kClimbLoadedVoltage = -11;
        public static final double kCurrentThresholdLoaded = 0.25;
        public static final double kMinOverCurrentTime = 0.15;
        public static final double kClimbMaxTime = 2;

        public static final double kThresholdBeforeRotateL4 = 20;
        public static final double kThresholdBeforeRotateL1To3 = 5;

        public static final ClosedLoopSlot kRegularSlot = ClosedLoopSlot.kSlot0;
        public static final ClosedLoopSlot kClimbSlot = ClosedLoopSlot.kSlot1;

        public static final double kMinHeightCoralTool = 6;
        public static final double kMinHeightAlgaeTool = 6;

        public static final double kThresholdForSlowdown = 20;

        public static enum StageLevel {
            Loading(30),
            Climb(-2),
            Stow(8),
            L1(19),
            L2(12),
            RemoveL2(37.5),
            L3(27),
            RemoveL3(54.5),
            L4(55);

            StageLevel(double value) {
                this.value = value;
            }

            public final double value; // in negative inches from 0 (floor level)
        }
    }

    public static final class TunnelCageConstants {
        public static final int kMotorId = 17;
        public static final int kCageSensorID = 9;

        public static final double kPositionThreshold = 0.02;

        public static final double kPFlywheel = 28;
        public static final double kIFlywheel = 0;
        public static final double kDFlywheel = 0;

        public static final double kVoltageCompensation = 11;
        public static final int kCurrentLimit = 15; // Amps

        public static final double fConversionPositionMeters = (7.0 / 225.0) * 0.0549 * Math.PI;
        public static final double fConversionVelocityMeters = fConversionPositionMeters / 60;

        public static final double kMinAutoInitTime = 0.15;
        public static final double kInitSpeed = -0.05;
        public static final double kThresholdMotorStopped = 0.0075;

        public static enum PinPosition {
            Init(0),
            Start(0.075),
            LockIntoCage(0.260),
            CageInHole(0.455);

            PinPosition(double value) {
                this.value = value;
            }

            public final double value;
        }

        public static final double kMoveCageSpeed = 0.15;
        public static final double kMoveCageAcceleration = 1;

        public static final double kManualSpeed = 0.1;

        public static final int kServoPwmPort = 8;
        public static final int kMaxPulseServo = 2500;
        public static final int kMinPulseServo = 500;
        public static final double kRetractedServoPosition = 0.16;
        public static final double kDeployedServoPosition = 0.5;
    }

    public static final class ManipulatorWheelsConstants {
        public static final int kMotorId = 15;
        public static final double kInPower = 0.8;
        public static final double kOutPower = -1;

        public static final double kVoltageCompensation = 11;
        public static final int kCurrentLimit = 40; // Amps

        public static final double kMinCoralIntakeTime = 0.1;
        public static final double kMaxCoralIntakeTime = 3;
        public static final double kCoralEjectTime = 0.75;
        public static final double kMinOverCurrentTime = 0.15;
        // RPM
        public static final double kThresholdMotorStopped = 500;

        // amps
        public static final double kStalledMotorCurrent = 14;
    }

    public static final class FieldConstants {
        /**
         * Varies based on competition location. Welded for regionals both in Canada &
         * USA, but check if welded/AndyMark for champs if applicable.
         */
        public static final AprilTagFieldLayout kFieldLayout = AprilTagFieldLayout
                .loadField(AprilTagFields.k2025ReefscapeWelded);

        public static final class ReefConstants {
            public static enum TargetSide {
                Left("L"),
                Right("R");

                TargetSide(String side) {
                    this.side = side;
                }

                public final String side;
            }

            public static final int[] aprilTagIDs = { 6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22 };

            /**
             * Tags for which that left/right targets should be flipped, to make more sense
             * for the driver
             */
            public static final ArrayList<Integer> leftRightFlippedTagIDs = new ArrayList<>(
                    Arrays.asList(9, 10, 11, 20, 21, 22));

            /** meters */
            public static final double kDistanceBetweenBranches = 0.33;

            /**
             * Left means when looking at the front of the tag.
             * <p>
             * Can be rotated with tags' orientation
             */
            public static final Translation2d kLeftOffsetFromTagFinalPosition = new Translation2d(
                    (DriveConstants.kRobotWidth / 2),
                    -(kDistanceBetweenBranches / 2) + CoralToolConstants.kToolDistanceFromCenterMeters);

            /**
             * Right means when looking at the front of the tag.
             * <p>
             * Using 0 degrees of rotation by default, so it can be rotated with tags'
             * orientation
             */
            public static final Translation2d kRightOffsetFromTagFinalPosition = new Translation2d(
                    (DriveConstants.kRobotWidth / 2),
                    (kDistanceBetweenBranches / 2) + CoralToolConstants.kToolDistanceFromCenterMeters);

            /** meters */
            public static final double kFinalApproachDistance = 0.3;

            public static final Translation2d kFinalApproachOffset = new Translation2d(kFinalApproachDistance, 0);

            public static final Rotation2d kScoringRotationOffset = Rotation2d.fromDegrees(90);

        }
    }

    public static final class AutoConstants {
        // public static final double kMaxSpeedMetersPerSecond = 4.75;
        // public static final double kMaxAccelerationMetersPerSecondSquared = 6;
        // public static final double kMaxAngularSpeedRadiansPerSecond = 1.5 * Math.PI;
        // public static final double kMaxAngularSpeedRadiansPerSecondSquared = 2 *
        // Math.PI;

        public static final double driveKP = 6.75;
        public static final double driveKI = 0;
        public static final double driveKD = 0;

        public static final double turnKP = 5;
        public static final double turnKI = 0;
        public static final double turnKD = 0;

        /* Constraint for the motion profilied robot angle controller */
        // public static final TrapezoidProfile.Constraints kThetaControllerConstraints
        // = new TrapezoidProfile.Constraints(
        // kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    /**
     * Standard deviations: X(m), Y(m), θ(rad)
     */
    public static final class PoseEstimationConstants {
        public static final double kMaxDistanceSingleTagEstimation = 1.5;

        public static final Matrix<N3, N1> kStateStdDevs = new Matrix<>(Nat.N3(), Nat.N1(),
                new double[] { 0.025, 0.025, 0.001 });
        // Basically dummy values given as a default, since we feed actual dynamic std
        // devs with each update from vision
        public static final Matrix<N3, N1> kVisionStdDevsDefault = new Matrix<>(Nat.N3(), Nat.N1(),
                new double[] { 0.05, 0.05, 100 });
        // because gyro is better than vision, we make σ_θ huge
        public static final Matrix<N3, N1> kVisionStdDevsPerMeterGlobal = new Matrix<>(Nat.N3(), Nat.N1(),
                new double[] { .04, .04, 100 });
        public static final Matrix<N3, N1> kVisionStdDevsBaselineGlobal = new Matrix<>(Nat.N3(), Nat.N1(),
                new double[] { .18, .18, 100 });
        public static final Matrix<N3, N1> kVisionStdDevsPerMeterCubedLocal = new Matrix<>(Nat.N3(), Nat.N1(),
                new double[] { .05, .05, 100 });
    }

    public static final class VisionConstants {
        public static final String kRightCameraName = "camera-a";
        public static final String kLeftCameraName = "camera-b";
        public static final double kMaxPoseAmbiguityAllowed = 0.2;

        /**
         * Transform from centre of robot facing forward (+X), at floor height, to
         * camera (facing outwards, +X in camera space)
         */
        public static final Transform3d kRobotToRightCameraTransform = new Transform3d(
                new Translation3d(.032, -0.338, .150),
                new Rotation3d(0, Degree.of(-26).in(Radians), Degree.of(-90).in(Radians)));

        /**
         * Transform from centre of robot facing forward (+X), at floor height, to
         * camera (facing outwards, +X in camera space)
         */
        public static final Transform3d kRobotToLeftCameraTransform = new Transform3d(
                new Translation3d(.032, 0.338, .150),
                new Rotation3d(0, Degree.of(-26).in(Radians), Degree.of(90).in(Radians)));
    }

    public static final class PDHConstants {
        public static final int kElevatorRightMotorChannel = 4;
        public static final int kElevatorLeftMotorChannel = 16;
        public static final int kManipulatorWheelsChannel = 14;
    }

    public static final class LedConstants {
        public static final int kPwmPort = 9;
        public static final int kNumLeds = 12;
        public static final Time kNoAllianceBlink = Seconds.of(.5);
        public static final Time kBreatheCycle = Seconds.of(2);

    }

    public static final double kPeriodicInterval = .02;

    public static final int kCANTimeout = 50;
    public static final int kPeriodicFrameTimeout = 500;
}
