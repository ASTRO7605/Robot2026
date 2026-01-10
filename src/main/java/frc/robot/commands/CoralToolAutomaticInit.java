package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralToolConstants;
import frc.robot.Constants.CoralToolConstants.CoralToolPosition;
import frc.robot.subsystems.CoralTool;

public class CoralToolAutomaticInit extends Command {
    private final CoralTool coralTool;
    private Timer timer = new Timer();

    public CoralToolAutomaticInit(CoralTool coralTool) {
        this.coralTool = coralTool;
        addRequirements(coralTool);
        // withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    @Override
    public void initialize() {
        coralTool.manualMotorVoltage(CoralToolConstants.kAutomaticInitVoltage);
        timer.restart();
    }

    @Override
    public void execute() {
        if (timer.hasElapsed(CoralToolConstants.kMinAutoInitTime) && coralTool.isMotorStopped()) {
            coralTool.setInitAsDone();
            coralTool
                    .resetEncoderPosition(CoralToolConstants.kLimitSwitchPosition + CoralToolConstants.kHardStopOffset);
        }
    }

    @Override
    public boolean isFinished() {
        return coralTool.isInitDone();
    }

    @Override
    public void end(boolean interrupted) {
        coralTool.goToPosition(CoralToolPosition.Stow);
    }
}