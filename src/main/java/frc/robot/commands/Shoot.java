package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterBase;

public class Shoot extends Command {

    private final Shooter shooter;
    private final Conveyor conveyor;
    private final ShooterBase shooterBase;

    public Shoot(Shooter shooter, Conveyor conveyor, ShooterBase shooterBase) {
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.shooterBase = shooterBase;
        addRequirements(shooter, conveyor, shooterBase);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // RobotContainer.m_shooter.setMotorSpeed(3000);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // RobotContainer.m_shooter.setMotorSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
