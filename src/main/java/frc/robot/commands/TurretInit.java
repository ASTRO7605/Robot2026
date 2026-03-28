package frc.robot.commands;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.ClimbConstants.climbLvl;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Tourelle;

public class TurretInit extends Command {
    private final Tourelle tourelle;
    private final Timer timer;

    public TurretInit(Tourelle tourelle) {
        this.tourelle = tourelle;
        this.timer = new Timer();
        addRequirements(tourelle);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        tourelle.setMotorPercentage(TurretConstants.kInitPercentage);
    }

    @Override
    public boolean isFinished() {
        return tourelle.isInitDone();
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            tourelle.setInitAsDone();
            tourelle.resetEncoderPosition();
            tourelle.setMotorPercentage(0);
        }
    }

}