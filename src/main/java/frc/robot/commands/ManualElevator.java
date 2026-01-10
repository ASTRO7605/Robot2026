package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ManualElevator extends Command {
    private final Elevator elevator;
    private double output;

    public ManualElevator(Elevator elevator, double output) {
        this.elevator = elevator;
        this.output = output;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        elevator.setMotorSpeed(output, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        elevator.keepPosition();
    }
    
}
