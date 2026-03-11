package frc.robot;

import java.util.Optional;

public record LimelightVisionResult(
                Optional<LimelightHelpers.PoseEstimate> mt1Estimate,
                Optional<LimelightHelpers.PoseEstimate> mt2Estimate,
                double closestTagDistance) {
}
