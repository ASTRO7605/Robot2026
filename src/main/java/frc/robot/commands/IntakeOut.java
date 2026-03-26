package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.intakePos;
import frc.robot.subsystems.Intake;

public class IntakeOut extends Command {
    private final Intake intake;

    public IntakeOut(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
        //withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }
   
    @Override
    public void execute(){
        intake.setManualMotorPercentage(0.1);
    }


    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted){
        intake.keepPosition();
    }



}