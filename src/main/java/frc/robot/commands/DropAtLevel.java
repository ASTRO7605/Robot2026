package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.AlgaeToolConstants.AlgaeToolPosition;
import frc.robot.Constants.CoralToolConstants.CoralToolPosition;
import frc.robot.Constants.ElevatorConstants.StageLevel;
import frc.robot.subsystems.AlgaeTool;
import frc.robot.subsystems.CoralTool;
import frc.robot.subsystems.Elevator;

public class DropAtLevel extends Command {
    private Elevator elevator;
    private CoralTool coralTool;
    private AlgaeTool algaeTool;

    private BooleanSupplier ejectButton;
    private BooleanSupplier dropAtLevelButton;
    private boolean hasTimerStarted;
    private double desiredPos;
    private Timer timer;

    private boolean hasButtonBeenReleased;
    private boolean hasCoralToolMoved;
    private boolean inAuto;
    private double thresholdRotate;

    public DropAtLevel(Elevator elevator, CoralTool coralTool,
            AlgaeTool algaeTool, double desiredPos, boolean inAuto, BooleanSupplier ejectButton,
            BooleanSupplier dropAtLevelButton) {
        this.elevator = elevator;
        this.coralTool = coralTool;
        this.algaeTool = algaeTool;
        this.desiredPos = desiredPos;
        this.inAuto = inAuto;

        this.ejectButton = ejectButton;
        this.dropAtLevelButton = dropAtLevelButton;
        this.timer = new Timer();
        addRequirements(algaeTool);
    }

    @Override
    public void initialize() {
        elevator.goToPosition(desiredPos, ElevatorConstants.kGeneralMaxVelocity,
                ElevatorConstants.kGeneralMaxAcceleration, ElevatorConstants.kRegularSlot);
        algaeTool.goToPosition(AlgaeToolPosition.Stow);

        timer.reset();
        hasTimerStarted = false;
        hasButtonBeenReleased = false;
        hasCoralToolMoved = false;
        thresholdRotate = desiredPos == StageLevel.L4.value ? ElevatorConstants.kThresholdBeforeRotateL4
                : ElevatorConstants.kThresholdBeforeRotateL1To3;
    }

    @Override
    public void execute() {
        if (ejectButton.getAsBoolean() && !hasTimerStarted) {
            timer.start();
            hasTimerStarted = true;
        }

        if (!dropAtLevelButton.getAsBoolean()) {
            hasButtonBeenReleased = true;
        }

        if (!hasCoralToolMoved
                && (Math.abs(elevator.getPosition() - desiredPos) <= thresholdRotate)) {
            double desiredToolPos;
            if (desiredPos == StageLevel.L4.value) {
                desiredToolPos = CoralToolPosition.L4Deposit.value;
            } else if (desiredPos == StageLevel.L1.value) {
                desiredToolPos = CoralToolPosition.L1Deposit.value;
            } else {
                desiredToolPos = CoralToolPosition.L2And3Deposit.value;
            }

            coralTool.goToPosition(desiredToolPos);
            hasCoralToolMoved = true;
        }
    }

    @Override
    public boolean isFinished() {
        return /* (timer.get() >= ManipulatorWheelsConstants.kCoralEjectTime) || */(hasButtonBeenReleased && !inAuto)
                || (inAuto && elevator.isAtPosition(desiredPos));
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted && !inAuto) {
            new StowSubsystems(coralTool, algaeTool, elevator, desiredPos == StageLevel.L1.value).schedule();
        }
    }

}
