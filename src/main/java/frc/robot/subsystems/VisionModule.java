package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Radian;

import java.io.OptionalDataException;
import java.lang.StackWalker.Option;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.OptionalInt;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.VisionResult;

public class VisionModule {
    private PhotonCamera m_camera;
    private String m_cameraName;
    private PhotonPoseEstimator m_photonGlobalPoseEstimator;
    private PhotonPoseEstimator m_photonLocalPoseEstimator;
    private Optional<Matrix<N3, N3>> m_cameraMatrix;
    private Optional<Matrix<N8, N1>> m_distCoeffs;
    private OptionalInt m_currentTagID;
    Supplier<Rotation2d> rotationProvider;

    private final Field2d m_globalVisionField = new Field2d();
    private final Field2d m_localVisionField = new Field2d();

    public VisionModule(String cameraName, Transform3d robotToCameraTransform, Supplier<Rotation2d> rotationProvider) {
        m_camera = new PhotonCamera(cameraName);
        this.rotationProvider = rotationProvider;

        m_photonGlobalPoseEstimator = new PhotonPoseEstimator(FieldConstants.kFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                robotToCameraTransform);

        m_photonGlobalPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        m_photonLocalPoseEstimator = new PhotonPoseEstimator(FieldConstants.kFieldLayout,
                PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
                robotToCameraTransform);

        m_photonLocalPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);

        m_cameraMatrix = m_camera.getCameraMatrix();
        m_distCoeffs = m_camera.getDistCoeffs();

        m_cameraName = cameraName;
        SmartDashboard.putData(cameraName + " Global Measurement", m_globalVisionField);
        SmartDashboard.putData(cameraName + " Local Measurement", m_localVisionField);
        m_currentTagID = OptionalInt.empty();

    }

    public OptionalInt getCurrentTagID() {
        return m_currentTagID;
    }

    public VisionResult getEstimatedPoses() {
        // We only return the last result after having cleared the queue
        Optional<EstimatedRobotPose> estimatedGlobal = Optional.empty();
        Optional<EstimatedRobotPose> estimatedLocal = Optional.empty();
        double distance = 0;
        Transform3d camToTarget = new Transform3d();
        int numTargets = 0;
        SmartDashboard.putNumber(m_cameraName + " Tag ID", m_currentTagID.isPresent() ? m_currentTagID.getAsInt() : 0);

        for (PhotonPipelineResult result : m_camera.getAllUnreadResults()) {
            if (!result.hasTargets()) {
                m_currentTagID = OptionalInt.empty();
                continue;
            }
            // Skip high-ambiguity single target readings
            if (result.targets.size() == 1
                    && result.targets.get(0).poseAmbiguity > VisionConstants.kMaxPoseAmbiguityAllowed) {
                m_currentTagID = OptionalInt.empty();
                continue;
            }

            PhotonTrackedTarget bestTarget = result.getTargets().get(0);
            for (var target : result.getTargets()) {
                if (target.getBestCameraToTarget().getTranslation().getNorm() < bestTarget.getBestCameraToTarget()
                        .getTranslation().getNorm()) {
                    bestTarget = target;
                }
            }

            m_currentTagID = OptionalInt.of(bestTarget.getFiducialId());

            estimatedGlobal = m_photonGlobalPoseEstimator.update(result, m_cameraMatrix, m_distCoeffs);

            m_photonLocalPoseEstimator.addHeadingData(Timer.getFPGATimestamp(), rotationProvider.get());
            estimatedLocal = m_photonLocalPoseEstimator.update(result, m_cameraMatrix, m_distCoeffs);

            camToTarget = result.getBestTarget().getBestCameraToTarget();
            distance = result.getBestTarget().bestCameraToTarget.getTranslation().toTranslation2d().getNorm();
            numTargets = result.targets.size();
        }

        // Convert to our struct

        if (!estimatedGlobal.isEmpty()) {
            var pose = estimatedGlobal.get().estimatedPose.toPose2d();
            if (Double.isNaN(pose.getX()) || Double.isNaN(pose.getY())
                    || Double.isNaN(pose.getRotation().getDegrees())) {
                estimatedGlobal = Optional.empty();
            } else {
                m_globalVisionField.setRobotPose(estimatedGlobal.get().estimatedPose.toPose2d());
            }
        }

        if (!estimatedLocal.isEmpty()) {
            var pose = estimatedLocal.get().estimatedPose.toPose2d();
            if (Double.isNaN(pose.getX()) || Double.isNaN(pose.getY())
                    || Double.isNaN(pose.getRotation().getDegrees())) {
                estimatedLocal = Optional.empty();
            } else {
                m_localVisionField.setRobotPose(estimatedLocal.get().estimatedPose.toPose2d());
            }
        }

        SmartDashboard.putNumber(m_cameraName + " Distance", distance);

        if (estimatedGlobal.isPresent() && estimatedLocal.isPresent()) {
            var delta = estimatedGlobal.get().estimatedPose.toPose2d()
                    .minus(estimatedLocal.get().estimatedPose.toPose2d());
            SmartDashboard.putString(m_cameraName + " pose delta", delta.toString());
        }

        return new VisionResult(estimatedGlobal, estimatedLocal, distance, numTargets);
    }
}
