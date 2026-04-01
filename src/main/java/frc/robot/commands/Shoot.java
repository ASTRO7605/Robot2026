package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterBase;

public class Shoot extends Command {

    private final Shooter shooter;
    private final Conveyor conveyor;
    private final ShooterBase shooterBase;
    private int shootCounter;
    private Command shootCommand;

    public Shoot(Shooter shooter, Conveyor conveyor, ShooterBase shooterBase) {
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.shooterBase = shooterBase;
        addRequirements(shooter, conveyor, shooterBase);
        
    }

    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void initialize() {
        shooter.setMotorSpeed(3000);
        shooterBase.ShooterBaseWheelsIn();

        shootCommand = new WaitCommand(.35). andThen(() -> conveyor.conveyorWheelsIn());
        shootCommand.schedule();
    }
    @Override
    public void end(boolean interrupted) {
        shooter.setMotorSpeed(0);
        conveyor.ConveyorWheelOff();
        shooterBase.ShooterBaseWheelOff();
        shootCommand.cancel();
    }

}
