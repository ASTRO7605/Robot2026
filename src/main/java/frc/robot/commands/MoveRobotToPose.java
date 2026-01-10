package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Base;

public class MoveRobotToPose extends Command {
    private final Base base;
    private Pose2d desiredPose;
    private HolonomicDriveController controller;

    public MoveRobotToPose(Base base, Pose2d desiredPose, TrapezoidProfile.Constraints rotationConstraints) {
        this.base = base;
        this.desiredPose = desiredPose;
        this.controller = new HolonomicDriveController(
                new PIDController(DriveConstants.kXYMovementP, DriveConstants.kXYMovementI,
                        DriveConstants.kXYMovementD),
                new PIDController(DriveConstants.kXYMovementP, DriveConstants.kXYMovementI,
                        DriveConstants.kXYMovementD),
                new ProfiledPIDController(DriveConstants.kRotationMovementP, DriveConstants.kRotationMovementI,
                        DriveConstants.kRotationMovementD, rotationConstraints));

        addRequirements(base);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }

}
