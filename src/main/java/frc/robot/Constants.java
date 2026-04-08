package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {

    public static final boolean kEnableVision = true;

    public static final class DriveConstants {
        public static final int kXboxControllerID = 2;
        public static final int kTurnStickID = 1;
        public static final int kThrottleStickID = 0;
        public static final int kTriggerID = 1;

        public static final double kControllerMovementDeadband = 0.1;
        public static final double kControllerRotationDeadband = 0.1;

        public static final double kTimeBeforeBrakeDisabled = 1;

        public static final int pigeonID = 1;
        public static final String kCanivoreBusId = "canivore";

        public static final COTSTalonFXSwerveConstants chosenModule = COTSTalonFXSwerveConstants.SDS.MK4i
                .KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double wheelBase = Units.inchesToMeters(23.75);
        public static final double trackWidth = Units.inchesToMeters(20.75);
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        // with bumpers
        public static final double kRobotWidth = Units.inchesToMeters(32.63);
        public static final double kRobotLength = Units.inchesToMeters(35.69);

        public static final Transform2d kTurretRobotPosition = new Transform2d(
                new Translation2d(Units.inchesToMeters(7.25), 0),
                new Rotation2d());

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
        public static final int angleCurrentLimit = 50;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 40;
        public static final int driveCurrentThreshold = 60;
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
        public static final double driveKS = 0.099238;
        public static final double driveKV = 2.5191;
        public static final double driveKA = 0.29158;

        // public static final double kXYMovementP = 5;
        // public static final double kXYMovementI = 0;
        // public static final double kXYMovementD = 0;

        // public static final double kRotationMovementP = 5;
        // public static final double kRotationMovementI = 0;
        // public static final double kRotationMovementD = 0;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double kMaxTeleopSpeed = 5.25;
        /** Radians per Second */
        public static final double kMaxTeleopRotateSpeed = 3 * Math.PI;

        public static final double kGeneralSpeedMulti = .8;
        public static final double kDriverSlowSpeed = 0.3;

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(47.46); // Anciennement
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
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(297.86); // Anciennement
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
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(298.92); // Anciennement
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
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-143.09); // Anciennement
            // -149.55
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                    angleMotorID,
                    canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants {
        public static final double driveKP = 6.75;
        public static final double driveKI = 0;
        public static final double driveKD = 0;

        public static final double turnKP = 5;
        public static final double turnKI = 0;
        public static final double turnKD = 0;
    }

    public static final double kPeriodicInterval = .02;

    public static final int kCANTimeout = 50;
    public static final int kPeriodicFrameTimeout = 500;

    ////////////////////////////////////////////////////////
    /// Constants des moteurs Neo Kraken pour les sous-systèmes autres que le
    //////////////////////////////////////////////////////// drivebase
    ////////////////////////////////////////////////////////

    public static final double kVoltageCompensation = 11;

    // constantes pour le climb
    public static final class ClimbConstants {
        public static final int climbMotorId = 16;
        public static final double kp = 1.10;
        public static final double ki = 0.0;
        public static final double kd = 0.0;
        public static final double maxVelocity = 25;
        public static final double maxAcceleration = 50;
        public static final int kCurrentLimit = 70;
        public static final double fPositionConversion = (1.25 * Math.PI) / 15.6; // en pouces par tour de moteur
        public static final double fVelocityConversion = fPositionConversion / 60;
        public static final double kSoftLimitForward = 15;
        public static final double feedforwards = 0; // tune with built climber
        public static final double kPositionThreshold = 0.25;

        // enum pour les positions de climb
        public static enum climbLvl {
            // in inches
            Stowed(2.8), // hauteur légale pour le climb
            Hang(0), // hauteur pour accrocher le robot à la barre
            Extended(14);// ~30 inch

            public final double position;

            climbLvl(double position) {
                this.position = position;
            }
        }

        public static final double kManualPercentage = 0.1;
        public static final double kInitPercentage = -0.1;
        public static final double kInitTimeDelaySeconds = 0.1;
        public static final double kStoppedMotorThreshold = 0.1;
        public static final double kInitPosition = 0;
    }

    // constantes pour les intakes
    public static final class IntakeConstants {
        public static final int rightIntakeMotorId = 9;
        public static final int kCurrentLimit = 50;
        public static final double kp = 0.015;
        public static final double ki = 0.0;
        public static final double kd = 0.04;
        public static final double kv = 0.0;
        // (Rotations –> °)
        public static final double fPositionConversion = 810 / 31;
        // RPM -> ° / s
        public static final double fVelocityConversion = fPositionConversion / 60;

        public static final double maxVelocity = 240;
        public static final double maxAcceleration = 700;
        public static final double kSoftLimitReverse = -142;
        public static final double kMaxAf = 0.65;
        public static final double kLimitSwitchPosition = 0; // degrees
        public static final double kAfOffset = 140; // degrés

        public static final double manualSpeed = 0.1;
        public static final double initSpeed = 0.08;

        public static final double wiggleTime = 1.0; // seconds

        public static final double kPositionThreshold = 3; // degrees

        // enum pour les positions de l'intake
        public static enum intakePos {
            // in degrees
            Stowed(-20),
            Down(-146),
            WiggleOut(-120),
            WiggleIn(-70);

            public final double position;

            intakePos(double position) {
                this.position = position;
            }
        }
    }

    // constantes pour le convoyeur
    public static final class ConveyorConstants {
        public static final int intakeRollerMotorId = 10;
        public static final int conveyorMotorId = 11;

        public static final int intakeRollerPdhChannel = 17;

        public static final int kCurrentLimit = 50;

        public static final double kp = 0.0;
        public static final double ki = 0.0;
        public static final double kd = 0.0;
        public static final double kv = 0.0;

        public static final double setConveyorInSpeed = 3000;
        public static final double setConveyorOutSpeed = -1000;

        public static final double conveyorTimeIn = 1.0; // seconds
        public static final double conveyorTimeOff = 0.15; // seconds

        public static final double manualPercentageInConveyor = 0.75;
        public static final double manualPercentageOutConveyor = -0.75;
        public static final double manualPercentageInIntake = 1;
        public static final double manualPercentageOutIntake = -1;

        // Motor RPM -> Roller RPM
        public static final double fPositionConversion = 1.0 / 4.0;
        public static final double fVelocityConversion = fPositionConversion / 60;

        public static final double kThresholdMotorStopped = 500;

    }

    // constantes pour le shooter
    public static final class ShooterConstants {
        public static final int rightShooterMotorId = 13;
        public static final int leftShooterMotorId = 14;
        public static final double kp = 0.0001;
        public static final double ki = 0.0;
        public static final double kd = 0.000175;
        public static final double kv = 0.0021;
        public static final int kCurrentLimit = 65;
        // RPM -> rotations par minute
        public static final double fPositionConversion = 1;
        public static final double fVelocityConversion = 1;

        public static final double kPredictPoseLatency = 0.1; // seconds

        public static final double kSpeedShootThreshold = 100;
    }

    // constantes pour la base du shooter
    public static final class ShooterBaseConstants {
        public static final int shooterBaseMotorId = 12;
        public static final double maxVelocity = 40;
        public static final double maxAcceleration = 60;
        public static final int kCurrentLimit = 50;

        public static final double kInSpeed = 0.5;
        public static final double kOutSpeed = -0.5;

        public static final double kThresholdMotorStopped = 500;
    }

    // constantes pour les tourelles
    public static final class TurretConstants {
        public static final int turretMotorId = 15;
        public static final double kp = 0.06;
        public static final double ki = 0.0;
        public static final double kd = 0.1;
        public static final int kCurrentLimit = 50;
        // (Rotations –> °)
        public static final double fPositionConversion = 9;
        // RPM -> degrés par seconde
        public static final double fVelocityConversion = fPositionConversion / 60;

        public static final double maxVelocity = 0;
        public static final double maxAcceleration = 0;
        public static final double kLimitSwitchPosition = 66; // degrees
        public static final double kSoftLimitReverse = -67;
        public static final double kInitPercentage = 0.075;

        public static final double kMaxSetpoint = 65;
        public static final double kMinSetpoint = -66;
        public static final double kExtremesThreshold = 15;

        public static final double kAngleShootThreshold = 1.5;
    }

    public static final class FieldConstants {
        public static final Translation2d kFullFieldCoords = new Translation2d(Units.inchesToMeters(652.22),
                Units.inchesToMeters(317.69));

        // position to switch between feeding / shooting
        public static final double kXPosFeed = Units.inchesToMeters(195);

        public static enum BluePositions {
            HUB(new Translation2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84))),
            DEPOT_DUMP(new Translation2d(Units.inchesToMeters(30), Units.inchesToMeters(287.69))),
            OUTPOST_DUMP(new Translation2d(Units.inchesToMeters(30), Units.inchesToMeters(30)));

            BluePositions(Translation2d coordinate) {
                this.translation2d = coordinate;
            }

            public final Translation2d translation2d;
        }

        /**
         * 
         * blue to
         * red or
         * red to blue
         */

        public static Translation2d flipPosColor(Translation2d pos) {
            // flip by 180 degrees
            return new Translation2d(kFullFieldCoords.getX() - pos.getX(),
                    kFullFieldCoords.getY() - pos.getY());
        }
    }

    /**
     * Standard deviations: X(m), Y(m), θ(rad)
     */
    public static final class PoseEstimationConstants {
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

        public static final String limelight4Name = "limelight-quatre";
        public static final String limelight3Name = "limelight-trois"; // IP: 10.76.5.13:5801
        public static final String limelight2Name = "limelight-deux";

        public static final Transform3d robotTolimelight4Transform = new Transform3d(new Translation3d(
                Units.inchesToMeters(13.06), Units.inchesToMeters(6.5), Units.inchesToMeters(11.12)),
                new Rotation3d(0, Units.degreesToRadians(26), Units.degreesToRadians(-41))); // 0,24,31
        public static final Transform3d robotTolimelight3Transform = new Transform3d(new Translation3d(
                Units.inchesToMeters(12.18), Units.inchesToMeters(-7.25), Units.inchesToMeters(11.31)),
                new Rotation3d(0, Units.degreesToRadians(24), Units.degreesToRadians(31))); // 0,24,31
        public static final Transform3d robotTolimelight2Transform = new Transform3d();

        public static final double kMaxPoseAmbiguityAllowed = 0.2;

        // see limelight docs
        public static enum LimelightIMUModes {
            EXTERNAL_ONLY(0),
            EXTERNAL_SEED(1),
            INTERNAL_ONLY(2),
            INTERNAL_MT1_ASSIST(3),
            INTERNAL_EXTERNAL_ASSIST(4);

            LimelightIMUModes(int value) {
                this.value = value;
            }

            public final int value;
        }

    }
}
