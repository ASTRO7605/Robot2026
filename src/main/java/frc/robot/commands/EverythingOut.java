package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterBase;

public class EverythingOut extends Command {
    private final Shooter shooter;
    private final Conveyor conveyor;
    private final ShooterBase shooterBase;

    public EverythingOut(Conveyor conveyor, Shooter shooter, ShooterBase shooterBase) {
    this.conveyor = conveyor;
    this.shooter = shooter;
    this.shooterBase = shooterBase;
    addRequirements(shooter, shooterBase, conveyor);
    }

    @Override
    public void initialize() {
    
    }
    
    
    @Override
    public void execute() {
        conveyor.conveyorWheelsOut();
        shooter.setMotorSpeed(-500);
        shooterBase.setMotorSpeed(-500);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.conveyorWheelsOff();
        shooter.setMotorSpeed(0);
        shooterBase.setMotorSpeed(0);
    }};
