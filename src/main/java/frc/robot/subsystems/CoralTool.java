package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkMaxConfigAccessor;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CoralToolConstants;
import frc.robot.Constants.CoralToolConstants.CoralToolPosition;
import frc.robot.commands.TrapezoidProfileMovement;
import frc.robot.utils.SparkMaxTunableMotorSystem;

public class CoralTool extends SubsystemBase implements SparkMaxTunableMotorSystem {
    private SparkMax motor = new SparkMax(CoralToolConstants.kMotorId, MotorType.kBrushless);
    private RelativeEncoder encoder = motor.getEncoder();
    private SparkClosedLoopController controller = motor.getClosedLoopController();
    private SparkLimitSwitch limitSwitch = motor.getForwardLimitSwitch();

    private boolean initDone = false;
    private boolean lastSwitchState = false;

    private boolean isElevatorRestrictionActive = true;
    private boolean isToolInRestrictedZone = false;

    private TrapezoidProfile.Constraints profileConstraints;

    public CoralTool() {
        motor.setCANTimeout(Constants.kCANTimeout);
        motor.setPeriodicFrameTimeout(Constants.kPeriodicFrameTimeout);

        var config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.inverted(false);
        config.closedLoop
                .p(CoralToolConstants.kP)
                .i(CoralToolConstants.kI)
                .d(CoralToolConstants.kD);

        config.voltageCompensation(CoralToolConstants.kVoltageCompensation);
        config.smartCurrentLimit(CoralToolConstants.kCurrentLimit);

        config.encoder.positionConversionFactor(CoralToolConstants.fPositionConversion);
        config.encoder.velocityConversionFactor(CoralToolConstants.fVelocityConversion);
        config.softLimit.reverseSoftLimit(CoralToolConstants.kReverseSoftLimit).reverseSoftLimitEnabled(true);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        profileConstraints = new TrapezoidProfile.Constraints(
                CoralToolConstants.kGeneralMaxVelocity,
                CoralToolConstants.kGeneralMaxAcceleration);
    }

    @Override
    public void periodic() {
        checkInit();
        // SmartDashboard.putBoolean(getSubsystem() + ".limitSwitch", limitSwitch.isPressed());
        SmartDashboard.putNumber(getSubsystem() + ".position", encoder.getPosition());
        SmartDashboard.putNumber(getSubsystem() + ".velocity", encoder.getVelocity());
        SmartDashboard.putBoolean(getSubsystem() + ".initDone", initDone);

        // SmartDashboard.putNumber("CoralTool calculatedFF", computeFF());
        // SmartDashboard.putBoolean(getSubsystem() + ".elevatorRestriction", isElevatorRestrictionActive);
        isToolInRestrictedZone = getPosition() <= CoralToolConstants.kMinPosWhenRestricted;
    }

    public void setElevatorRestriction(boolean state) {
        isElevatorRestrictionActive = state;
    }

    public boolean isToolInRestrictedZone() {
        return isToolInRestrictedZone;
    }

    public double computeFF() {
        return (CoralToolConstants.kMaxFF * Math.cos(Math.toRadians(getPosition())));
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    public Command goToPosition(CoralToolPosition level) {
        return goToPosition(level.value);
    }

    public Command goToPosition(double target) {
        if (!initDone) {
            return new InstantCommand(() -> System.out.println("Not initialized. Ignoring command"));
        }

        if (isElevatorRestrictionActive && (target < CoralToolConstants.kMinPosWhenRestricted)) {
            return new PrintCommand("Cannot go to pos when restricted");
        }

        var command = new TrapezoidProfileMovement(motor, target, profileConstraints, this::computeFF,
                ClosedLoopSlot.kSlot0);
        command.addRequirements(this);
        command.schedule();
        return command;
    }

    public boolean isMotorStopped() {
        return Math.abs(encoder.getVelocity()) < CoralToolConstants.kThresholdMotorStopped;
    }

    private boolean movingTowardZero() {
        return encoder.getVelocity() > 0;
    }

    public boolean isAtPosition(CoralToolPosition p) {
        return Math.abs(encoder.getPosition() - p.value) < CoralToolConstants.kPositionThreshold;
    }

    public void resetEncoderPosition(double position) {
        encoder.setPosition(position);
    }

    public boolean isLimitSwitchActivated() {
        return limitSwitch.isPressed();
    }

    private void checkInit() {
        final var switchState = limitSwitch.isPressed();
        if (switchState == true
                && lastSwitchState == false
                && movingTowardZero()) {
            initDone = true;
            resetEncoderPosition(CoralToolConstants.kLimitSwitchPosition);
        }
        lastSwitchState = switchState;
    }

    public boolean isInitDone() {
        return initDone;
    }

    public void setInitAsDone() {
        initDone = true;
    }

    // No FF is applied
    public void manualMotorVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    public void setMotorVoltage(double voltage) {
        if (isElevatorRestrictionActive
                && (voltage < 0 && isToolInRestrictedZone)) {
            voltage = 0;
        }
        controller.setReference(voltage, ControlType.kVoltage, ClosedLoopSlot.kSlot0, computeFF());
    }

    public void setMotorSpeed(double speed) {
        if (isElevatorRestrictionActive
                && (speed < 0 && isToolInRestrictedZone)) {
            speed = 0;
        }
        controller.setReference(speed, ControlType.kDutyCycle, ClosedLoopSlot.kSlot0, computeFF());
    }

    public void keepPosition() {
        controller.setReference(getPosition(), ControlType.kPosition, ClosedLoopSlot.kSlot0, computeFF());
    }

    public void freezeAllMotorFunctions() {
        motor.stopMotor();
    }

    @Override
    public SparkMaxConfigAccessor getMotorConfig() {
        return motor.configAccessor;
    }

    @Override
    public TrapezoidProfile.Constraints getProfileConstraints() {
        return profileConstraints;
    }

    @Override
    public void apply(SparkMaxConfig config, TrapezoidProfile.Constraints constraints) {
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        this.profileConstraints = constraints;
    }
}