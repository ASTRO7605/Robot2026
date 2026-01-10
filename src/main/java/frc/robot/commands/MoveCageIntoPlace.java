package frc.robot.commands;

import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.TunnelCageConstants;
import frc.robot.Constants.TunnelCageConstants.PinPosition;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.TunnelCage;

public class MoveCageIntoPlace extends Command {
    private final TunnelCage tunnelCage;
    private final Base base;
    private final Led led;

    public MoveCageIntoPlace(TunnelCage tunnelCage, Base base, Led led) {
        this.tunnelCage = tunnelCage;
        this.base = base;
        this.led = led;
        addRequirements(base);
    }

    @Override
    public void initialize() {
        tunnelCage.goToPositionTrapezoid(PinPosition.CageInHole.value);
        
    }

    @Override
    public void execute() {
        base.driveRobotRelativeChassisSpeeds(new ChassisSpeeds(-TunnelCageConstants.kMoveCageSpeed, 0, 0));
    }

    @Override
    public boolean isFinished() {
        return tunnelCage.isAtPosition(PinPosition.CageInHole.value);
    }

    @Override
    public void end(boolean interrupted) {
        base.driveRobotRelativeChassisSpeeds(new ChassisSpeeds());
        if (!interrupted) {
            new ParallelDeadlineGroup(new WaitCommand(1.5),
                    new SimpleLedPattern(led, LEDPattern.solid(Color.kGreen).blink(Second
                            .of(0.1))))
                    .schedule();
        }
    }
}
