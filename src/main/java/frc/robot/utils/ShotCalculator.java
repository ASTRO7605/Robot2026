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
    private double lastTargetDistance = 0;
    private double targetAngle = 0;
    private final boolean usingShootOnMove = true;

    // table de calcul de la vitesse en fonction de la distance
    private final InterpolatingDoubleTreeMap distanceToRpm = new InterpolatingDoubleTreeMap();
    // table de calcul du temps de vol (Time Of Flight) en fonction de la distance
    private final InterpolatingDoubleTreeMap tofFromDistance = new InterpolatingDoubleTreeMap();
    // table de calcul de la distance en fonction du TOF
    private final InterpolatingDoubleTreeMap distanceFromTof = new InterpolatingDoubleTreeMap();
    // table de calcul de l'angle de tir en fonction de la distance
    private final InterpolatingDoubleTreeMap angleFromDistance = new InterpolatingDoubleTreeMap();

    private void addDataToTofTables(double distance, double tof) {
        tofFromDistance.put(distance, tof);
        distanceFromTof.put(tof, distance);
    }

    public double getRpmForDistance() {
        return distanceToRpm.get(lastTargetDistance);
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double getAngleForDistance() {
        return angleFromDistance.get(lastTargetDistance);
    }

    private ShotCalculator() {
        turretToTargetField = new Field2d();
        // put distance / speed couples
        // distanceToRpm.put(1.74, 2500.0);
        // distanceToRpm.put(2.57, 2800.0);
        // distanceToRpm.put(3.09, 2800.0);
        // distanceToRpm.put(3.16, 2800.0);
        // distanceToRpm.put(3.3, 2900.0);
        // distanceToRpm.put(3.4, 3000.0);
        // distanceToRpm.put(4.0, 3300.0);
        // distanceToRpm.put(4.18, 3300.0);
        // distanceToRpm.put(4.75, 3500.0);
        // distanceToRpm.put(5.1, 3700.0);

        distanceToRpm.put(1.75, 2750.0);
        distanceToRpm.put(2.0, 2775.0);
        distanceToRpm.put(2.27, 2825.0);
        distanceToRpm.put(2.5, 2900.0);
        distanceToRpm.put(2.75, 2950.0);
        distanceToRpm.put(3.2, 3265.0);
        distanceToRpm.put(3.6, 3450.0);
        distanceToRpm.put(4.0, 3600.0);
        distanceToRpm.put(4.4, 3750.0);
        distanceToRpm.put(4.8, 4050.0);
        // put distance / tof couples
        addDataToTofTables(1.82, 0.79);
        addDataToTofTables(2.12, 0.84);
        addDataToTofTables(2.42, 0.9);
        addDataToTofTables(2.73, 0.95);
        addDataToTofTables(3.16, 1.03);
        addDataToTofTables(3.48, 1.11);
        addDataToTofTables(3.95, 1.23);
        addDataToTofTables(4.35, 1.37);
        addDataToTofTables(4.83, 1.43);

        SmartDashboard.putNumber("Test Shooter Speeds", 0);
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
        // offset the pose by a bit to account for system latency
        Pose2d projectedRobotPose = robotPose.exp(robotSpeeds.toTwist2d(ShooterConstants.kPredictPoseLatency));
        SmartDashboard.putString("projected robot pose", projectedRobotPose.toString());
        Pose2d projectedTurretPose = projectedRobotPose.transformBy(DriveConstants.kTurretRobotPosition);

        // METHOD 1: vector subtraction to simulate a static shot from where we are
        // convert robot relative speeds to field relative
        ChassisSpeeds fieldSpeeds = usingShootOnMove
                ? ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, robotPose.getRotation())
                : new ChassisSpeeds();

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
        Translation2d resultVector = new Translation2d(desiredBallSpeed,
                turretToTargetTranslation.getAngle());

        // result vector - robot speed vector = shot vector
        Translation2d shotVector = resultVector.minus(new Translation2d(turretXSpeed,
                turretYSpeed));

        // calculate wheel parameters
        double estimatedDistance = targetDistance;
        lastTargetDistance = targetDistance;

        if (usingShootOnMove) {
            for (int i = 0; i < 3; i++) {
                // calculate the speed of the ball from this estimated distance
                double estimatedTof = tofFromDistance.get(estimatedDistance);
                double resultingSpeed = estimatedDistance / estimatedTof;

                // try to reduce the error by interpolating
                double ratio = shotVector.getNorm() / resultingSpeed;
                estimatedDistance *= ratio;
            }
        }

        double wheelSpeeds = distanceToRpm.get(estimatedDistance);
        // wheelSpeeds = SmartDashboard.getNumber("Test Shooter Speeds", 0);
        SmartDashboard.putNumber("turretCalculatedSpeeds", wheelSpeeds);

        // turret angle
        Rotation2d turretAngle = shotVector.getAngle().minus(projectedRobotPose.getRotation());
        targetAngle = turretAngle.getDegrees();
        SmartDashboard.putNumber("turretToTargetAngle", turretAngle.getDegrees());

        currentShotInfo = new ShotInfo(turretAngle, wheelSpeeds);
    }
}
