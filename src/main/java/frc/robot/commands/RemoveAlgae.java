package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.AlgaeToolConstants.AlgaeToolPosition;
import frc.robot.Constants.CoralToolConstants.CoralToolPosition;
import frc.robot.Constants.ElevatorConstants.StageLevel;
import frc.robot.subsystems.AlgaeTool;
import frc.robot.subsystems.CoralTool;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ManipulatorWheels;

public class RemoveAlgae extends Command {
    private final Elevator elevator;
    private final CoralTool coralTool;
    private final AlgaeTool algaeTool;
    private final ManipulatorWheels manipulatorWheels;

    private StageLevel desiredLevel;
    private BooleanSupplier removeAlgaeButton;

    private boolean hasAlgaeToolComeDown;
    private boolean hasButtonBeenReleased;

    public RemoveAlgae(Elevator elevator, CoralTool coralTool, AlgaeTool algaeTool,
            ManipulatorWheels manipulatorWheels, BooleanSupplier removeAlgaeButton) {
        this.elevator = elevator;
        this.coralTool = coralTool;
        this.algaeTool = algaeTool;
        this.manipulatorWheels = manipulatorWheels;
        this.removeAlgaeButton = removeAlgaeButton;

        addRequirements(algaeTool, manipulatorWheels);
    }

    @Override
    public void initialize() {
        coralTool.goToPosition(CoralToolPosition.Stow);
        algaeTool.goToPosition(AlgaeToolPosition.Stow);

        if (elevator.isAtPosition(StageLevel.L4)) {
            desiredLevel = StageLevel.RemoveL3;
        } else {
            desiredLevel = StageLevel.RemoveL2;
        }

        elevator.goToPosition(desiredLevel, ElevatorConstants.kGeneralMaxVelocity,
                ElevatorConstants.kGeneralMaxAcceleration, ElevatorConstants.kRegularSlot);

        hasAlgaeToolComeDown = false;
        hasButtonBeenReleased = false;
    }

    @Override
    public void execute() {
        if (elevator.isAtPosition(desiredLevel) && !hasAlgaeToolComeDown) {
            manipulatorWheels.turnOnWheelsOut();
            algaeTool.goToPosition(AlgaeToolPosition.Low);

            hasAlgaeToolComeDown = true;
        }

        if (!removeAlgaeButton.getAsBoolean()) {
            hasButtonBeenReleased = true;
        }
    }

    @Override
    public boolean isFinished() {
        return hasButtonBeenReleased;
    }

    @Override
    public void end(boolean interrupted) {
        manipulatorWheels.turnOffWheels();

        if (!interrupted) {
            new StowSubsystems(coralTool, algaeTool, elevator, false).schedule();
        }
    }

}
