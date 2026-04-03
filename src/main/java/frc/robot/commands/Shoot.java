package frc.robot.commands;

import com.fasterxml.jackson.databind.ser.std.TokenBufferSerializer;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterBase;
import frc.robot.subsystems.Tourelle;
import frc.robot.utils.ShotCalculator;

public class Shoot extends Command {

    private final Shooter shooter;
    private final Conveyor conveyor;
    private final ShooterBase shooterBase;
    private final Tourelle tourelle;

    private boolean hasStartedShooting;

    public Shoot(Shooter shooter, Conveyor conveyor, ShooterBase shooterBase, Tourelle tourelle) {
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.shooterBase = shooterBase;
        this.tourelle = tourelle;

        addRequirements(shooter, conveyor, shooterBase, tourelle);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void initialize() {
        hasStartedShooting = false;
        conveyor.conveyorWheelsIn();
    }

    @Override
    public void execute() {
        var currentShotInfo = ShotCalculator.getInstance().getShotInfo();
        shooter.setMotorSpeed(currentShotInfo.wheelSpeeds());
        boolean valid = tourelle.requestTurretAngle(currentShotInfo.turretAngle());

        if (!valid) {
            shooterBase.ShooterBaseWheelOff();
            hasStartedShooting = false;
        } else if (!hasStartedShooting) {
            if (shooter.areWheelsAtSpeed(currentShotInfo.wheelSpeeds())
                    && tourelle.isAtTarget(currentShotInfo.turretAngle())) {
                shooterBase.ShooterBaseWheelsIn();
                hasStartedShooting = true;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setMotorSpeed(0);
        conveyor.ConveyorWheelOff();
        shooterBase.ShooterBaseWheelOff();
        tourelle.requestTurretAngle(new Rotation2d(0));
    }

}
