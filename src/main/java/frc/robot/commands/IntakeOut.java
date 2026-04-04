package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.intakePos;
import frc.lib.math.Conversions;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Conveyor;

public class IntakeOut extends Command {
    private final Intake intake;
    private final Conveyor conveyor;

    public IntakeOut(Intake intake, Conveyor conveyor) {
        this.intake = intake;
        this.conveyor = conveyor;
        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        intake.goToPosition(intakePos.Down);
    }

    @Override
    public void execute() {
        conveyor.conveyorWheelsIn();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.conveyorWheelsOff();
    }

}