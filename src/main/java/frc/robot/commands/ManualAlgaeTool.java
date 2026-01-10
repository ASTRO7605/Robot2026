package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeTool;

public class ManualAlgaeTool extends Command {
    private final AlgaeTool algaeTool;
    private double output;

    public ManualAlgaeTool(AlgaeTool algaeTool, double output) {
        this.algaeTool = algaeTool;
        this.output = output;

        addRequirements(algaeTool);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        algaeTool.setMotorSpeed(output);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        algaeTool.setMotorSpeed(0);
        algaeTool.keepPosition();
    }

}
