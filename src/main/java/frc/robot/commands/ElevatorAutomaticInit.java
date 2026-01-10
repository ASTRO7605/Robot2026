package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.StageLevel;
import frc.robot.subsystems.Elevator;

public class ElevatorAutomaticInit extends Command {
    private final Elevator elevator;

    public ElevatorAutomaticInit(Elevator elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
        // withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    @Override
    public void initialize() {
        elevator.setMotorSpeed(ElevatorConstants.kInitSpeed, true);
    }

    @Override
    public boolean isFinished() {
        return elevator.isInitDone();
    }

    @Override
    public void end(boolean interrupted) {
        elevator.goToPosition(StageLevel.Stow, ElevatorConstants.kGeneralMaxVelocity,
                ElevatorConstants.kGeneralMaxAcceleration, ElevatorConstants.kRegularSlot);
    }
}
