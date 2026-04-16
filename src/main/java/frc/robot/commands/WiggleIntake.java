package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.intakePos;

import frc.robot.subsystems.Intake;

public class WiggleIntake extends Command {
    private final Intake m_intake;
    private Timer WiggleTimer = new Timer();
    private boolean isWiggleIn;

    public WiggleIntake(Intake intake) {
        this.m_intake = intake;
    }

    @Override
    public void initialize() {
        m_intake.goToPosition(intakePos.WiggleIn);
        WiggleTimer.reset();
        WiggleTimer.start();
        isWiggleIn = true;
    }

    @Override
    public void execute() {
        if ((m_intake.isAtTarget(intakePos.WiggleIn.position) || WiggleTimer.get() >= IntakeConstants.wiggleTime)
                && isWiggleIn) {
            WiggleTimer.reset();
            WiggleTimer.start();
            m_intake.goToPosition(intakePos.WiggleOut);
            isWiggleIn = false;
        } else if ((m_intake.isAtTarget(intakePos.WiggleOut.position)
                || WiggleTimer.get() >= IntakeConstants.wiggleTime) && !isWiggleIn) {
            WiggleTimer.reset();
            WiggleTimer.start();
            m_intake.goToPosition(intakePos.WiggleIn);
            isWiggleIn = true;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.safeStop();
    }

}
