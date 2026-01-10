package frc.robot.commands;

import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.TunnelCageConstants;
import frc.robot.Constants.TunnelCageConstants.PinPosition;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.TunnelCage;

public class LockIntoCage extends Command {
    private final TunnelCage tunnelCage;
    private final Led led;

    private boolean hasCageBeenSeen;
    public LockIntoCage(TunnelCage tunnelCage, Led led) {
        this.tunnelCage = tunnelCage;
        this.led = led;

        addRequirements(tunnelCage);
    }

    @Override
    public void initialize() {
        hasCageBeenSeen = false;
        new SimpleLedPattern(led, LEDPattern.solid(Color.kOrange)).schedule();
    }

    @Override
    public void execute() {
        if (tunnelCage.isCageInTunnel()) {
            hasCageBeenSeen = true;
            tunnelCage.goToPosition(PinPosition.LockIntoCage.value);
        }
    }

    @Override
    public boolean isFinished() {
        return hasCageBeenSeen;
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            new ParallelDeadlineGroup(new WaitCommand(1.5),
                    new SimpleLedPattern(led, LEDPattern.solid(Color.kGreen).blink(Second
                            .of(0.1))))
                    .schedule();
        }
    }
}
