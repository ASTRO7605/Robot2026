package frc.robot.commands;

import static edu.wpi.first.units.Units.Second;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AlgaeToolConstants.AlgaeToolPosition;
import frc.robot.Constants.CoralToolConstants.CoralToolPosition;
import frc.robot.Constants.ElevatorConstants.StageLevel;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ManipulatorWheelsConstants;
import frc.robot.Constants.PDHConstants;
import frc.robot.subsystems.AlgaeTool;
import frc.robot.subsystems.CoralTool;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.ManipulatorWheels;
import frc.robot.utils.PDH;

public class LoadFromStation extends Command {
    private ManipulatorWheels manipulatorWheels;
    private Elevator elevator;
    private CoralTool coralTool;
    private AlgaeTool algaeTool;
    private Led led;

    private boolean isOverCurrent;
    private boolean hasLedsBeenScheduled;

    private Timer timer;
    private BooleanSupplier isLoadButtonPressed;
    // private boolean isOverCurrent;

    public LoadFromStation(ManipulatorWheels manipulatorWheels, Elevator elevator, CoralTool coralTool,
            AlgaeTool algaeTool, Led led, BooleanSupplier isLoadButtonPressed) {
        this.manipulatorWheels = manipulatorWheels;
        this.elevator = elevator;
        this.coralTool = coralTool;
        this.algaeTool = algaeTool;
        this.led = led;

        this.timer = new Timer();
        this.isLoadButtonPressed = isLoadButtonPressed;
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
        hasLedsBeenScheduled = false;
    }

    @Override
    public void execute() {
        final var current = PDH.getInstance().getChannelCurrent(
                PDHConstants.kManipulatorWheelsChannel);
        SmartDashboard.putNumber("manipulator wheel PDH current", current);

        if ((current >= ManipulatorWheelsConstants.kStalledMotorCurrent)) {
            if (!isOverCurrent) {
                timer.restart();
                isOverCurrent = true;
            }
        } else {
            isOverCurrent = false;
            timer.stop();
        }

        if (isOverCurrent && timer.get() >= ManipulatorWheelsConstants.kMinOverCurrentTime && !hasLedsBeenScheduled) {
            manipulatorWheels.turnOffWheels();
            new ParallelDeadlineGroup(new WaitCommand(1.5),
                    new SimpleLedPattern(led, LEDPattern.solid(Color.kTurquoise).blink(Second
                            .of(0.1))))
                    .schedule();
            hasLedsBeenScheduled = true;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        manipulatorWheels.turnOffWheels();

        if (interrupted) {
            // If interrupted by something else than the button being released
            if (isLoadButtonPressed.getAsBoolean()) {
                return;
            }
        }
        new StowSubsystems(coralTool, algaeTool, elevator, false).schedule();
    }

}
