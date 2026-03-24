package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;

// singleton to contain shot information
public class ShotCalculator {
    private static ShotCalculator instance = null;
    private ShotInfo currentShotInfo;
    private Field2d turretToTargetField;

    // table de calcul de la vitesse en fonction de la distance
    private final InterpolatingDoubleTreeMap distanceTable = new InterpolatingDoubleTreeMap();

    private ShotCalculator() {
        turretToTargetField = new Field2d();
        // put distance / speed couples
        distanceTable.put(0.0, 0.0);

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

        var turretToTarget = target.minus(robotPose.transformBy(DriveConstants.kTurretRobotPosition).getTranslation());
        turretToTargetField.setRobotPose(robotPose.getX(), robotPose.getY(), turretToTarget.getAngle());

        SmartDashboard.putNumber("turretToTargetDistance", turretToTarget.getNorm());

        var turretAngle = turretToTarget.getAngle().minus(robotPose.getRotation());
        SmartDashboard.putNumber("turretToTargetAngle", turretAngle.getDegrees());

        currentShotInfo = new ShotInfo(turretAngle.getDegrees(), 0);
    }
}
