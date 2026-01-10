package frc.robot.commands;

import org.ejml.data.ElementLocation;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ManipulatorWheelsConstants;
import frc.robot.Constants.PDHConstants;
import frc.robot.Constants.TunnelCageConstants;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.TunnelCage;
import frc.robot.utils.PDH;

public class ClimbCage extends Command {
    private final Elevator elevator;
    private final TunnelCage tunnelCage;
    private final Base base;

    private boolean isOverCurrent;
    private boolean hasSpeedChanged;
    private Timer overCurrentTimer;

    private Timer motorsLoadedTimer;

    private boolean isClimbDone;

    public ClimbCage(Elevator elevator, TunnelCage tunnelCage, Base base) {
        this.elevator = elevator;
        this.tunnelCage = tunnelCage;
        this.base = base;
        overCurrentTimer = new Timer();
        motorsLoadedTimer = new Timer();
        addRequirements(elevator, base);
    }

    @Override
    public void initialize() {
        tunnelCage.setServoPosition(TunnelCageConstants.kDeployedServoPosition);
        base.setModulesFacingForward();
        isOverCurrent = false;
        hasSpeedChanged = false;
        isClimbDone = false;
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("climbLoadedTimer", motorsLoadedTimer.get());

        if (isClimbDone) {
            elevator.freezeAllMotorFunctions();
            return;
        }

        if (elevator.isBottomSwitchActivated() || motorsLoadedTimer.get() >= ElevatorConstants.kClimbMaxTime) {
            isClimbDone = true;
        }

        if ((PDH.getInstance().getChannelCurrent(
                PDHConstants.kElevatorRightMotorChannel) >= ElevatorConstants.kCurrentThresholdLoaded)) {
            if (!isOverCurrent) {
                overCurrentTimer.restart();
                isOverCurrent = true;
            }
        } else {
            isOverCurrent = false;
            overCurrentTimer.stop();
        }

        if (isOverCurrent && overCurrentTimer.get() >= ManipulatorWheelsConstants.kMinOverCurrentTime
                && !hasSpeedChanged) {
            elevator.setCurrentLimit(ElevatorConstants.kClimbCurrentLimit);
            hasSpeedChanged = true;
            motorsLoadedTimer.restart();
        }

        if (!hasSpeedChanged) {
            elevator.setMotorSpeed(ElevatorConstants.kClimbFreeSpeed, true);
        } else {
            elevator.setMotorVoltage(ElevatorConstants.kClimbLoadedVoltage, false);
        }

    }

    @Override
    public boolean isFinished() {
        // don't kill the command so that the wheels can stay facing forward
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setCurrentLimit(ElevatorConstants.kRegularCurrentLimit);
    }
}