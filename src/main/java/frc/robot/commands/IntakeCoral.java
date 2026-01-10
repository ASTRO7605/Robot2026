package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ManipulatorWheelsConstants;
import frc.robot.Constants.PDHConstants;
import frc.robot.subsystems.ManipulatorWheels;
import frc.robot.utils.PDH;

public class IntakeCoral extends Command {
    private ManipulatorWheels wheels;

    private Timer timer = new Timer();
    private boolean isOverCurrent;
    private boolean stopWithCurrent;

    public IntakeCoral(ManipulatorWheels wheels, boolean stopWithCurrent) {
        addRequirements(wheels);
        this.wheels = wheels;

        this.stopWithCurrent = stopWithCurrent;
    }

    @Override
    public void initialize() {
        wheels.turnOnWheelsIn();
        timer.reset();
        isOverCurrent = false;
    }

    @Override
    public boolean isFinished() {
        if ((PDH.getInstance().getChannelCurrent(
                PDHConstants.kManipulatorWheelsChannel) >= ManipulatorWheelsConstants.kStalledMotorCurrent)) {
            if (!isOverCurrent) {
                timer.restart();
                isOverCurrent = true;
            }
        } else {
            isOverCurrent = false;
            timer.stop();
        }

        return (stopWithCurrent && isOverCurrent && (timer.get() >= ManipulatorWheelsConstants.kMinOverCurrentTime));
    }

    @Override
    public void end(boolean interrupted) {
        wheels.turnOffWheels();
    }
}