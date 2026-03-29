package frc.robot.utils;

import java.util.Vector;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;

// singleton to contain shot information
public class ShotCalculator {
    private static ShotCalculator instance = null;
    private ShotInfo currentShotInfo;
    private Field2d turretToTargetField;

    // table de calcul de la vitesse en fonction de la distance
    private final InterpolatingDoubleTreeMap RPMFromDistance = new InterpolatingDoubleTreeMap();
    // table de calcul du temps de vol (Time Of Flight) en fonction de la distance
    private final InterpolatingDoubleTreeMap tofFromDistance = new InterpolatingDoubleTreeMap();
    // table de calcul de la distance en fonction du TOF
    private final InterpolatingDoubleTreeMap distanceFromTof = new InterpolatingDoubleTreeMap();

    private void addDataToTofTables(double distance, double tof) {
        tofFromDistance.put(distance, tof);
        distanceFromTof.put(tof, distance);
    }

    private ShotCalculator() {
        turretToTargetField = new Field2d();
        // put distance / speed couples
        RPMFromDistance.put(1.0, 3000.0);

        // put distance / tof couples
        addDataToTofTables(1.0, 1.0);

        SmartDashboard.putData("Turret to Target", turretToTargetField);
    }

    public static ShotCalculator getInstance() {
        if (instance == null) {
            instance = new ShotCalculator();
        }

        return instance;
    }

    public ShotInfo getShotInfo() {
        return currentShotInfo;
    }

    public void updateShotInfo(ChassisSpeeds robotSpeeds, Pose2d robotPose, Translation2d target) {
        turretToTargetField.getObject("Target").setPose(target.getX(), target.getY(),
                new Rotation2d());

        // offset the pose by a bit to account for system latency
        Pose2d projectedRobotPose = robotPose.exp(robotSpeeds.toTwist2d(ShooterConstants.kPredictPoseLatency));
        SmartDashboard.putString("projected robot pose", projectedRobotPose.toString());
        Pose2d projectedTurretPose = projectedRobotPose.transformBy(DriveConstants.kTurretRobotPosition);

        // METHOD 1: vector subtraction to simulate a static shot from where we are

        // convert robot relative speeds to field relative
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, robotPose.getRotation());

        // calculate the x and y speed of the turret
        Translation2d robotToTurretInField = DriveConstants.kTurretRobotPosition.getTranslation()
                .rotateBy(robotPose.getRotation());

        double turretXSpeed = fieldSpeeds.vxMetersPerSecond
                - fieldSpeeds.omegaRadiansPerSecond * robotToTurretInField.getY();
        double turretYSpeed = fieldSpeeds.vyMetersPerSecond
                + fieldSpeeds.omegaRadiansPerSecond * robotToTurretInField.getX();

        // result vector (speed and angle)
        Translation2d turretToTargetTranslation = target.minus(projectedTurretPose.getTranslation());
        double targetDistance = turretToTargetTranslation.getNorm();
        SmartDashboard.putNumber("turretToTargetDistance", targetDistance);

        double desiredBallSpeed = targetDistance
                / tofFromDistance.get(targetDistance);
        Translation2d resultVector = new Translation2d(desiredBallSpeed, turretToTargetTranslation.getAngle());

        // result vector - robot speed vector = shot vector
        Translation2d shotVector = resultVector.minus(new Translation2d(turretXSpeed, turretYSpeed));

        // calculate wheel parameters
        double estimatedDistance = targetDistance;

        for (int i = 0; i < 3; i++) {
            // calculate the speed of the ball from this estimated distance
            double estimatedTof = tofFromDistance.get(estimatedDistance);
            double resultingSpeed = estimatedDistance / estimatedTof;

            // try to reduce the error by interpolating
            double ratio = shotVector.getNorm() / resultingSpeed;
            estimatedDistance *= ratio;
        }

        double wheelSpeeds = RPMFromDistance.get(estimatedDistance);
        // turret angle
        Rotation2d turretAngle = shotVector.getAngle().minus(projectedRobotPose.getRotation());
        SmartDashboard.putNumber("turretToTargetAngle", turretAngle.getDegrees());

        currentShotInfo = new ShotInfo(turretAngle, wheelSpeeds);
    }
}
