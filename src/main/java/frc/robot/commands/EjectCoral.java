package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ManipulatorWheelsConstants;
import frc.robot.subsystems.ManipulatorWheels;

public class EjectCoral extends Command {
    private ManipulatorWheels m_manipulator;
    private Timer timer = new Timer();

    public EjectCoral(ManipulatorWheels manipulator) {
        addRequirements(manipulator);
        this.m_manipulator = manipulator;
        this.timer = new Timer();
    }

    @Override
    public void initialize() {
        m_manipulator.turnOnWheelsOut();
        timer.restart();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(ManipulatorWheelsConstants.kCoralEjectTime);
    }

    @Override
    public void end(boolean interrupted) {
        m_manipulator.turnOffWheels();
    }
}