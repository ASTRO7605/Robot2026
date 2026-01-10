package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TunnelCage;
import frc.robot.Constants.TunnelCageConstants;
import frc.robot.Constants.TunnelCageConstants.PinPosition;

public class TunnelCageAutomaticInit extends Command {
    private final TunnelCage tunnelCage;

    private final Timer timer;

    public TunnelCageAutomaticInit(TunnelCage tunnelCage) {
        this.tunnelCage = tunnelCage;
        timer = new Timer();
        addRequirements(tunnelCage);
        // withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    @Override
    public void initialize() {
        tunnelCage.setMotorSpeed(TunnelCageConstants.kInitSpeed);
        timer.restart();
    }

    @Override
    public boolean isFinished() {
        return tunnelCage.isMotorStopped() && (timer.get() >= TunnelCageConstants.kMinAutoInitTime);
    }

    @Override
    public void end(boolean interrupted) {
        tunnelCage.setMotorSpeed(0);
        tunnelCage.resetEncoderPosition(PinPosition.Init.value);
        tunnelCage.setInitAsDone();
        tunnelCage.goToPosition(PinPosition.Start.value);
    }
}