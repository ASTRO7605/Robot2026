package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterBase;
import frc.robot.subsystems.Tourelle;
import frc.robot.utils.ShotCalculator;
import edu.wpi.first.wpilibj.Timer;

public class FailSafeShoot extends Command {

    private final Shooter shooter;
    private final Conveyor conveyor;
    private final ShooterBase shooterBase;
    private final Timer conveyorTimer = new Timer();
    private boolean hasStartedShooting;
    private boolean conveyorOn;
    private BooleanSupplier intakeCmdActive;

    public FailSafeShoot(Shooter shooter, Conveyor conveyor, ShooterBase shooterBase,
            BooleanSupplier intakeCmdActive) {
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.shooterBase = shooterBase;
        this.intakeCmdActive = intakeCmdActive;

        addRequirements(shooter, conveyor, shooterBase);

    }

    @Override
    public void initialize() {
        hasStartedShooting = false;
        conveyor.conveyorWheelsIn();
        conveyorTimer.start();
        conveyorOn = true;
    }

    @Override
    public void execute() {
        var currentShotInfo = ShotCalculator.getInstance().getShotInfo();
        shooter.setMotorSpeed(currentShotInfo.wheelSpeeds());

        if (!intakeCmdActive.getAsBoolean()) {
            if (conveyorOn) {
                if (conveyorTimer.hasElapsed(ConveyorConstants.conveyorTimeIn)) {
                    conveyor.conveyorWheelsOff();
                    conveyorOn = false;
                    conveyorTimer.restart();
                }

            }

            else {
                if (conveyorTimer.hasElapsed(ConveyorConstants.conveyorTimeOff)) {
                    conveyor.conveyorWheelsIn();
                    conveyorOn = true;
                    conveyorTimer.restart();
                }
            }
        } else {
            conveyor.conveyorWheelsIn();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setMotorSpeed(0);
        conveyor.conveyorWheelsOff();
        shooterBase.ShooterBaseWheelOff();
    }

}
