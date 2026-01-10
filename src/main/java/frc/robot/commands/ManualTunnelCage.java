package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TunnelCage;

public class ManualTunnelCage extends Command {
    private final TunnelCage tunnelCage;
    private double output;

    public ManualTunnelCage(TunnelCage tunnelCage, double output) {
        this.tunnelCage = tunnelCage;
        this.output = output;

        addRequirements(tunnelCage);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        tunnelCage.setMotorSpeed(output);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        tunnelCage.setMotorSpeed(0);
        // tunnelCage.keepPosition();
    }

}
