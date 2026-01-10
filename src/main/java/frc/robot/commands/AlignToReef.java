package frc.robot.commands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants.ReefConstants;
import frc.robot.Constants.FieldConstants.ReefConstants.TargetSide;
import frc.robot.subsystems.Base;

public class AlignToReef extends Command {
    private final Base base;
    // weird thing happens with flipping if they're not different
    private TargetSide initialTargetSide;
    private TargetSide targetSide;

    private Command pathCommand;
    private boolean hasPathBeenScheduled;

    public AlignToReef(Base base, TargetSide targetSide) {
        this.base = base;
        this.initialTargetSide = targetSide;
    }

    @Override
    public void initialize() {
        hasPathBeenScheduled = false;
        targetSide = initialTargetSide;
        pathCommand = new Command() {
        };
    }

    @Override
    public void execute() {
        if (!hasPathBeenScheduled) {
            var optTagID = base.getMainTagIDSeenReefSide();
            if (optTagID.isPresent()) {
                var tagID = optTagID.getAsInt();
                // flip target side for driver
                // if (ReefConstants.leftRightFlippedTagIDs.contains(tagID)) {
                // if (targetSide == TargetSide.Right) {
                // targetSide = TargetSide.Left;
                // } else if (targetSide == TargetSide.Left) {
                // targetSide = TargetSide.Right;
                // }
                // }

                var pathConstraints = new PathConstraints(DriveConstants.kReefPathfindMaxSpeed,
                        DriveConstants.kReefPathfindMaxAccel, DriveConstants.kReefPathfindMaxRotateSpeed,
                        DriveConstants.kReefPathfindMaxRotateAccel);

                var path = base.getReefApproachPath(tagID, targetSide.side);
                pathCommand = AutoBuilder.pathfindThenFollowPath(path, pathConstraints);
                base.setPathfindingFlag(true);
                pathCommand.schedule();

                hasPathBeenScheduled = true;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return pathCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        pathCommand.cancel();
        base.setPathfindingFlag(false);
        base.drive(new Translation2d(), 0, false);
    }

}
