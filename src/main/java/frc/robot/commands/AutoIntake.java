package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeToolConstants.AlgaeToolPosition;
import frc.robot.Constants.CoralToolConstants.CoralToolPosition;
import frc.robot.Constants.ElevatorConstants.StageLevel;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ManipulatorWheelsConstants;
import frc.robot.Constants.PDHConstants;
import frc.robot.subsystems.AlgaeTool;
import frc.robot.subsystems.CoralTool;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ManipulatorWheels;
import frc.robot.utils.PDH;

public class AutoIntake extends Command {
    private ManipulatorWheels manipulatorWheels;
    private Elevator elevator;
    private CoralTool coralTool;
    private AlgaeTool algaeTool;

    private boolean isOverCurrent;
    private boolean isCommandFinished;

    private Timer timer;
    // private boolean isOverCurrent;

    public AutoIntake(ManipulatorWheels manipulatorWheels, Elevator elevator, CoralTool coralTool,
            AlgaeTool algaeTool) {
        this.manipulatorWheels = manipulatorWheels;
        this.elevator = elevator;
        this.coralTool = coralTool;
        this.algaeTool = algaeTool;

        this.timer = new Timer();
        addRequirements(manipulatorWheels, algaeTool);
    }

    @Override
    public void initialize() {
        elevator.goToPosition(StageLevel.Loading, ElevatorConstants.kGeneralMaxVelocity,
                ElevatorConstants.kGeneralMaxAcceleration, ElevatorConstants.kRegularSlot);
        coralTool.goToPosition(CoralToolPosition.Loading);
        algaeTool.goToPosition(AlgaeToolPosition.Stow);
        manipulatorWheels.turnOnWheelsIn();
        timer.reset();
        isOverCurrent = false;
        isCommandFinished = false;
    }

    @Override
    public void execute() {
        if ((PDH.getInstance().getChannelCurrent(
                PDHConstants.kManipulatorWheelsChannel) >= ManipulatorWheelsConstants.kStalledMotorCurrent)) {
            if (!isOverCurrent) {
                timer.restart();
                isOverCurrent = true;
            }
        } else {
            isOverCurrent = false;
            timer.stop();
        }

        if (isOverCurrent && timer.get() >= ManipulatorWheelsConstants.kMinOverCurrentTime) {
            manipulatorWheels.turnOffWheels();
            isCommandFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return isCommandFinished;
    }

    @Override
    public void end(boolean interrupted) {
        manipulatorWheels.turnOffWheels();
    }
}
