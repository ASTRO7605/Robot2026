package frc.robot.subsystems;

import java.util.Optional;
import java.util.OptionalInt;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightVisionResult;

public class LimelightVisionModule {
    private String m_cameraName;
    private OptionalInt m_closestTagID;
    private Supplier<Rotation2d> m_rotationProvider;

    private final Field2d m_mt1Field = new Field2d();
    private final Field2d m_mt2Field = new Field2d();

    public LimelightVisionModule(String cameraName, Transform3d robotToCameraTransform,
            Supplier<Rotation2d> rotationProvider) {
        m_rotationProvider = rotationProvider;
        m_cameraName = cameraName;
        m_closestTagID = OptionalInt.empty();

        // configure camera position for position estimation
        var robotToCamRotation = robotToCameraTransform.getRotation();
        LimelightHelpers.setCameraPose_RobotSpace(cameraName, robotToCameraTransform.getX(),
                robotToCameraTransform.getY(), robotToCameraTransform.getZ(), Math.toDegrees(robotToCamRotation.getX()),
                Math.toDegrees(robotToCamRotation.getY()), Math.toDegrees(robotToCamRotation.getZ()));

        SmartDashboard.putData(cameraName + " MT1", m_mt1Field);
        SmartDashboard.putData(cameraName + " MT2", m_mt2Field);
    }

    public OptionalInt getMainTagIDSeen() {
        return m_closestTagID;
    }

    public LimelightVisionResult getEstimatedPoses() {
        Optional<PoseEstimate> estimatedMt1 = Optional.empty();
        Optional<PoseEstimate> estimatedMt2 = Optional.empty();
        double closestDistance = 0;

        LimelightHelpers.SetRobotOrientation(m_cameraName, m_rotationProvider.get().getDegrees(), 0, 0, 0, 0, 0);

        var newMt1Pose = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_cameraName);
        var newMt2Pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_cameraName);

        if ((newMt2Pose == null) || (newMt2Pose.tagCount == 0) || (newMt2Pose.tagCount == 1
                && newMt2Pose.rawFiducials[0].ambiguity > VisionConstants.kMaxPoseAmbiguityAllowed)) {
            m_closestTagID = OptionalInt.empty(); // no tags or single tag with too high ambiguity
        } else {
            var closestTarget = newMt2Pose.rawFiducials[0];
            for (RawFiducial target : newMt2Pose.rawFiducials) {
                if (target.distToCamera < closestTarget.distToCamera) {
                    closestTarget = target;
                }
            }

            m_closestTagID = OptionalInt.of(closestTarget.id);

            closestDistance = closestTarget.distToCamera;

            var delta = newMt2Pose.pose
                    .minus(newMt1Pose.pose);
            SmartDashboard.putString(m_cameraName + " Pose Delta", delta.toString());

            m_mt1Field.setRobotPose(newMt1Pose.pose);
            m_mt2Field.setRobotPose(newMt2Pose.pose);

            estimatedMt1 = Optional.of(newMt1Pose);
            estimatedMt2 = Optional.of(newMt2Pose);
        }

        SmartDashboard.putNumber(m_cameraName + " Tag ID", m_closestTagID.isPresent() ? m_closestTagID.getAsInt() : 0);
        SmartDashboard.putNumber(m_cameraName + " Closest Tag Distance", closestDistance);

        return new LimelightVisionResult(estimatedMt1, estimatedMt2, closestDistance);
    }

    public void setIMUMode(VisionConstants.LimelightIMUModes mode) {
        LimelightHelpers.SetIMUMode(m_cameraName, mode.value);
    }
}
