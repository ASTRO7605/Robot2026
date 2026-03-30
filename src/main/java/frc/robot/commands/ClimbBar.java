package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ClimbConstants.climbLvl;
import frc.robot.subsystems.Climb;

public class ClimbBar extends Command {

    private final Climb climb;
    public ClimbBar(Climb climb) {
        this.climb = climb;
        addRequirements(climb);
    }

    @Override
    public void execute() {
        climb.goToPosition(climbLvl.Hang);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        climb.keepPosition();
        climb.notPrepareToClimb();
    }
}
