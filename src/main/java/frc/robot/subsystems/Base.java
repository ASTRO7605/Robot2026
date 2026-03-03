package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PoseEstimationConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.FieldConstants.ReefConstants;
import frc.robot.utils.SparkMaxMotorTuner;
import frc.robot.VisionResult;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.OptionalInt;
import java.util.stream.IntStream;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Base extends SubsystemBase {
    private final Field2d field = new Field2d();
    private final Pigeon2 m_gyro;
    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final SwerveModule[] m_swerveMods;
    final CANBus canbus = new CANBus(DriveConstants.kCanivoreBusId);

    private final Field2d m_robotField = new Field2d();
    private final Field2d m_targetField = new Field2d();

    private final SendableChooser<String> m_targetChooser = new SendableChooser<>();

    private Alliance m_allianceColor = Alliance.Red;

    /** radians */
    private double m_gyroOffset = 0.0;

    private boolean m_currentlyPathfinding = false;
    private boolean m_currentlyIntaking = false;
    private boolean m_drivingInFieldRelative = true;

    private boolean m_globalVisionMode = true;

    /**
     * Use AprilTag ID + L/R as key for which tag to align to
     * <p>
     * Example: "9R"
     */
    private final Map<String, Pose2d> m_reefFinalPositions;

    /**
     * Use AprilTag ID + L/R as key for which tag to align to
     * <p>
     * Example: "9R"
     */
    //private final Map<String, PathPlannerPath> m_reefFinalApproachPaths;

    /**
     * 0: left camera (station)
     * <p>
     * 1: right camera(reef)
     */
    private ArrayList<VisionModule> m_visionModules = new ArrayList<>();
    private ArrayList<VisionModule> m_reefVisionModules = new ArrayList<>();
    private ArrayList<VisionModule> m_intakeVisionModules = new ArrayList<>();

    public Base() {
        m_gyro = new Pigeon2(Constants.DriveConstants.pigeonID, canbus);

        m_gyro.getConfigurator().apply(new Pigeon2Configuration());
        m_gyro.setYaw(0);
        SmartDashboard.putData("Field", field);
        Optional<RobotConfig> robotConfig;

        try {
            robotConfig = Optional.of(RobotConfig.fromGUISettings());
        } catch (Exception e) {
            robotConfig = Optional.empty();
            e.printStackTrace();
        }

        m_swerveMods = new SwerveModule[] {
                new SwerveModule(0, DriveConstants.Mod0.constants),
                new SwerveModule(1, DriveConstants.Mod1.constants),
                new SwerveModule(2, DriveConstants.Mod2.constants),
                new SwerveModule(3, DriveConstants.Mod3.constants)
        };

        m_poseEstimator = new SwerveDrivePoseEstimator(
                DriveConstants.swerveKinematics, m_gyro.getRotation2d(),
                getModulePositions(),
                new Pose2d(new Translation2d(0, 0), new Rotation2d(0)),
                PoseEstimationConstants.kStateStdDevs,
                PoseEstimationConstants.kVisionStdDevsDefault);

        if (Constants.kEnableVision) {
            // var leftCamera = new VisionModule(VisionConstants.kLeftCameraName,
            // VisionConstants.kRobotToLeftCameraTransform, this::getHeading);
            // var rightCamera = new VisionModule(VisionConstants.kRightCameraName,
            // VisionConstants.kRobotToRightCameraTransform, this::getHeading);
            // m_visionModules.addAll(Arrays.asList(leftCamera, rightCamera));
            // m_reefVisionModules.addAll(Arrays.asList(rightCamera));
            // m_intakeVisionModules.addAll(Arrays.asList(leftCamera));
        }

        SmartDashboard.putData("Robot Measurement", m_robotField);
        SmartDashboard.putData("Target Pose", m_targetField);

        AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) ->

                driveRobotRelativeChassisSpeeds(speeds), // Method that will drive the robot
                                                         // given ROBOT RELATIVE
                                                         // ChassisSpeeds. Also optionally
                                                         // outputs individual module
                                                         // feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
                                                // holonomic drive trains
                        new PIDConstants(AutoConstants.driveKP, AutoConstants.driveKI, AutoConstants.driveKD), // Translation
                                                                                                               // PID
                                                                                                               // constants
                        new PIDConstants(AutoConstants.turnKP, AutoConstants.turnKI, AutoConstants.turnKD) // Rotation
                                                                                                           // PID
                                                                                                           // constants
                ),
                robotConfig.get(), // The robot configuration
                this::shouldPathsFlip,
                this // Reference to this subsystem to set requirements
        );
        m_drivingInFieldRelative = true;
        m_gyro.reset();

        m_reefFinalPositions = new HashMap<>();

        // test april tag targets
        for (var tagID : ReefConstants.aprilTagIDs) {
            String strID = Integer.toString(tagID);
            m_targetChooser.addOption(strID, strID);
        }
        m_targetChooser.setDefaultOption("7", "7");
        SmartDashboard.putData(m_targetChooser);

        SmartDashboard.putString("Target Reef Tag Side", "R");
    }

    public boolean shouldPathsFlip() {
        if (m_currentlyPathfinding) {
            return false;
        }
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

    public void setPathfindingFlag(boolean newValue) {
        m_currentlyPathfinding = newValue;
    }

    public void setIntakingFlag(boolean newValue) {
        m_currentlyIntaking = newValue;
    }

    public void drive(Translation2d translation, double rotation, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.DriveConstants.swerveKinematics.toSwerveModuleStates(
                m_drivingInFieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        Rotation2d.fromRadians(m_gyro.getRotation2d().getRadians() - m_gyroOffset))
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.DriveConstants.kMaxTeleopSpeed);

        for (SwerveModule mod : m_swerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.m_moduleNumber], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.DriveConstants.kMaxTeleopSpeed);

        for (SwerveModule mod : m_swerveMods) {
            mod.setDesiredState(desiredStates[mod.m_moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : m_swerveMods) {
            states[mod.m_moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : m_swerveMods) {
            positions[mod.m_moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveRobotRelativeChassisSpeeds(ChassisSpeeds speeds) {
        var swerveModuleStates = DriveConstants.swerveKinematics.toSwerveModuleStates(speeds);

        setModuleStates(swerveModuleStates);
    }

    public void setPose(Pose2d pose) {
        m_poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        m_poseEstimator.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        m_poseEstimator.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble());
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : m_swerveMods) {
            mod.resetToAbsolute();
        }
    }

    public void setModulesFacingForward() {
        for (var module : m_swerveMods) {
            module.setDesiredState(new SwerveModuleState(0, new Rotation2d(0)), false);
        }
    }

    public void stopWheels() {
        for (var module : m_swerveMods) {
            module.setDesiredState(new SwerveModuleState(0, new Rotation2d(0)), false);
        }
    }

    public void changeFieldDrivingMode() {
        m_drivingInFieldRelative = !m_drivingInFieldRelative;
    }

    public void switchToRobotRelative() {
        m_drivingInFieldRelative = false;
    }

    public void switchToFieldRelative() {
        m_drivingInFieldRelative = true;
    }

    public void resetGyroOffset(boolean usePoseEstimator) {
        if (!usePoseEstimator) {
            m_gyroOffset = m_gyro.getRotation2d().getRadians();
        } else {
            double currentHeading = m_poseEstimator.getEstimatedPosition().getRotation().getRadians();
            double currentGyroAngle = m_gyro.getRotation2d().getRadians();
            double targetHeading = (m_allianceColor == DriverStation.Alliance.Blue) ? 0
                    : Math.toRadians(180);

            double error = targetHeading - currentHeading;

            m_gyroOffset = currentGyroAngle + error;
        }
    }

    public Optional<Pose2d> getAveragePoseFromCameras() {
        List<Pose2d> poses = new ArrayList<>();
        for (VisionModule module : m_visionModules) {
            var measurement = module.getEstimatedPoses().estimatedGlobalPose();
            measurement.ifPresent(result -> poses.add(result.estimatedPose.toPose2d()));
        }

        if (poses.isEmpty()) {
            return Optional.empty();
        }

        Translation2d averageTranslation = new Translation2d();
        Rotation2d averageRotation = new Rotation2d();
        for (Pose2d pose : poses) {
            averageTranslation = averageTranslation.plus(pose.getTranslation());
            averageRotation = averageRotation.plus(pose.getRotation());
        }

        averageTranslation = averageTranslation.div(poses.size());
        averageRotation = averageRotation.div(poses.size());

        return Optional.of(new Pose2d(averageTranslation, averageRotation));
    }

    public void setNeutralMode(NeutralModeValue neutralMode) {
        for (SwerveModule mod : m_swerveMods) {
            mod.setNeutralMode(neutralMode);
        }
    }

    /**
     * Returns the main tag ID that is seen by all cameras on reef side or only one
     * of them. If no tags seen or two don't see the same, empty.
     */
    public OptionalInt getMainTagIDSeenReefSide() {
        ArrayList<Integer> tagIDs = new ArrayList<>();
        for (var vis : m_reefVisionModules) {
            var visTagID = vis.getCurrentTagID();
            if (visTagID.isPresent()) {
                tagIDs.add(visTagID.getAsInt());
            }
        }

        if (tagIDs.size() == 0) {
            return OptionalInt.empty();
        }

        var firstTag = tagIDs.get(0);
        for (var tag : tagIDs) {
            if (tag != firstTag) {
                return OptionalInt.empty();
            }
        }

        // filter out non-reef tags
        if (!IntStream.of(ReefConstants.aprilTagIDs).anyMatch(x -> x == firstTag)) {
            return OptionalInt.empty();
        }

        return OptionalInt.of(firstTag);
    }

    public void updateVisionEstimate() {
        for (VisionModule vis : m_visionModules) {
            VisionResult o_result = vis.getEstimatedPoses();
            var global = o_result.numTargets() > 1;
            var result = global ? o_result.estimatedGlobalPose() : o_result.estimatedLocalPose();
            double distance = o_result.averageTagDistance();

            // filter out empty results or cameras not looking at reef when pathfinding
            if (result.isEmpty() || (m_currentlyPathfinding && !m_reefVisionModules.contains(vis))
                    || (m_currentlyIntaking && !m_intakeVisionModules.contains(vis))) {
                continue;
            }

            // if only one tag, but too far, use global to trust less
            if (!global && distance >= PoseEstimationConstants.kMaxDistanceSingleTagEstimation) {
                result = o_result.estimatedGlobalPose();
            }

            var stdDevs = global ? PoseEstimationConstants.kVisionStdDevsPerMeterGlobal.times(distance)
                    .plus(PoseEstimationConstants.kVisionStdDevsBaselineGlobal)
                    : PoseEstimationConstants.kVisionStdDevsPerMeterCubedLocal.times(distance * distance * distance);

            Pose2d measurement2d = result.get().estimatedPose.toPose2d();

            m_poseEstimator.addVisionMeasurement(measurement2d, result.get().timestampSeconds,
                    stdDevs);
        }
    }

    @Override
    public void periodic() {
        // update pose estimator with vision, if applicable
        updateVisionEstimate();
        // update pose estimator with gyro and swerve
        m_poseEstimator.update(getGyroYaw(), getModulePositions());
        {
            var all = DriverStation.getAlliance();
            if (all.isPresent()) {
                m_allianceColor = all.get();
            }
        }

        for (SwerveModule mod : m_swerveMods) {
            SmartDashboard.putNumber("Mod " + mod.m_moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.m_moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.m_moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }

        field.setRobotPose(m_poseEstimator.getEstimatedPosition());
        m_robotField.setRobotPose(m_poseEstimator.getEstimatedPosition());
        m_targetField.setRobotPose(m_reefFinalPositions.getOrDefault(
                m_targetChooser.getSelected() + SmartDashboard.getString("Target Reef Tag Side", "R"), new Pose2d()));
        // m_targetField.setRobotPose(m_reefFinalApproachPaths.get(
        // m_targetChooser.getSelected() + SmartDashboard.getString("Target Reef Tag
        // Side", "R"))
        // .getStartingHolonomicPose().get());

        SmartDashboard.putNumber("RobotOrientation", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees());
        SmartDashboard.putNumber("RobotHeading", getHeading().getRadians());
        // SmartDashboard.putNumber("RobotXPosition",
        // m_poseEstimator.getEstimatedPosition().getX());
        // SmartDashboard.putNumber("RobotYPosition",
        // m_poseEstimator.getEstimatedPosition().getY());
        SmartDashboard.putNumber("Reef Tag ID",
                getMainTagIDSeenReefSide().isPresent() ? getMainTagIDSeenReefSide().getAsInt() : 0);

        SmartDashboard.putNumber("CANivore Usage", canbus.getStatus().BusUtilization);
    }

    public void setGlobalVisionMode() {
        m_globalVisionMode = true;
    }

    public void setLocalVisionMode() {
        m_globalVisionMode = false;
    }

    public Pose2d getTargetPose(int tagID, String side) {
        return m_reefFinalPositions.getOrDefault(tagID + side, null);
    }
}