package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

public record VisionResult(
        Optional<EstimatedRobotPose> estimatedGlobalPose,
        Optional<EstimatedRobotPose> estimatedLocalPose,
        double averageTagDistance,
        int numTargets) {
}
