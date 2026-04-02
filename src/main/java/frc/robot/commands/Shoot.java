package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterBase;
import frc.robot.subsystems.Tourelle;

public class Shoot extends Command {

    private final Shooter shooter;
    private final Conveyor conveyor;
    private final ShooterBase shooterBase;
    private final Tourelle tourelle;
    private Command shootCommand;

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
        shooter.turnOnShooter();
        shooterBase.ShooterBaseWheelsIn();
        tourelle.goToAngle();
    //Pour laisser le temps à la roue de shooter de monter en vitesse avant de faire avancer les balles
        shootCommand = new WaitCommand(.35). andThen(() -> conveyor.conveyorWheelsIn());
        shootCommand.schedule();
    }
    @Override
    public void end(boolean interrupted) {
        shooter.setMotorSpeed(0);
        conveyor.ConveyorWheelOff();
        shooterBase.ShooterBaseWheelOff();
        tourelle.safeStop();
        shootCommand.cancel();
    }

}
