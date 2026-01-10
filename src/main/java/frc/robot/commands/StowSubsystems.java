package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.AlgaeToolConstants.AlgaeToolPosition;
import frc.robot.Constants.CoralToolConstants.CoralToolPosition;
import frc.robot.Constants.ElevatorConstants.StageLevel;
import frc.robot.subsystems.AlgaeTool;
import frc.robot.subsystems.CoralTool;
import frc.robot.subsystems.Elevator;

public class StowSubsystems extends Command {
    private final CoralTool coralTool;
    private final AlgaeTool algaeTool;
    private final Elevator elevator;

    private final boolean waitForCoralTool;
    private boolean hasCoralToolReached;

    public StowSubsystems(CoralTool coralTool, AlgaeTool algaeTool, Elevator elevator, boolean waitForCoralTool) {
        this.coralTool = coralTool;
        this.algaeTool = algaeTool;
        this.elevator = elevator;

        this.waitForCoralTool = waitForCoralTool;

        hasCoralToolReached = false;
        addRequirements(algaeTool);
    }

    @Override
    public void initialize() {
        coralTool.goToPosition(CoralToolPosition.Stow);
    }

    @Override
    public void execute() {
        if ((coralTool.isAtPosition(CoralToolPosition.Stow) && !hasCoralToolReached) || !waitForCoralTool) {
            hasCoralToolReached = true;
            algaeTool.goToPosition(AlgaeToolPosition.Stow);
            elevator.goToPosition(StageLevel.Stow, ElevatorConstants.kGeneralMaxVelocity,
                    ElevatorConstants.kGeneralMaxAcceleration, ElevatorConstants.kRegularSlot);
        }
    }

    @Override
    public boolean isFinished() {
        return hasCoralToolReached || !waitForCoralTool;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
