package frc.robot.commands;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ClimbConstants.climbLvl;
import frc.robot.subsystems.Climb;

public class ClimberInit extends Command {
    private final Climb climb;
    private final Timer timer;

    public ClimberInit(Climb climb) {
        this.climb = climb;
        this.timer = new Timer();
        addRequirements(climb);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        climb.setMotorPercentage(ClimbConstants.kInitPercentage, true);
    }

    @Override
    public boolean isFinished() {
        return timer.get() > ClimbConstants.kInitTimeDelaySeconds && climb.isMotorStopped();
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            climb.setInitDone();
            climb.resetEncoderPosition();
            climb.goToPosition(climbLvl.Stowed, ClimbConstants.maxVelocity,
            ClimbConstants.maxAcceleration); 
        }
    }

}
