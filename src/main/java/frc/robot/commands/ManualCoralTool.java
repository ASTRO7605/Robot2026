package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralTool;

public class ManualCoralTool extends Command {
    private final CoralTool coralTool;
    private double speed;

    public ManualCoralTool(CoralTool coralTool, double speed) {
        this.coralTool = coralTool;
        this.speed = speed;

        addRequirements(coralTool);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        coralTool.setMotorSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        coralTool.keepPosition();
    }

}