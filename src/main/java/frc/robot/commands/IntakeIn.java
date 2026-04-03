package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants.intakePos;
import frc.robot.subsystems.Intake;

public class IntakeIn extends Command {
    private final Intake intake;

    public IntakeIn(Intake intake) {
        this.intake = intake;
    }

    @Override
    public void initialize() {
        intake.goToPosition(intakePos.Stowed);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
