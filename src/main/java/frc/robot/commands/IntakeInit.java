package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.intakePos;
import frc.robot.subsystems.Intake;

public class IntakeInit extends Command {
    private final Intake intake;

    public IntakeInit(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.setManualMotorPercentage(IntakeConstants.initSpeed);
    }

    @Override
    public boolean isFinished() {
        return intake.isInitDone();
    }

    @Override
    public void end(boolean interrupted) {
        intake.setManualMotorPercentage(0);
    }

}
