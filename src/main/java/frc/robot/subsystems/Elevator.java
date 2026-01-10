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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PDHConstants;
import frc.robot.Constants.ElevatorConstants.StageLevel;
import frc.robot.commands.TrapezoidProfileMovement;
import frc.robot.utils.PDH;
import frc.robot.utils.SparkMaxTunableMotorSystem;

public class Elevator extends SubsystemBase implements SparkMaxTunableMotorSystem {

    private SparkMax leftMotor = new SparkMax(ElevatorConstants.kLeftMotorId, MotorType.kBrushless);
    private SparkMax rightMotor = new SparkMax(ElevatorConstants.kRightMotorId, MotorType.kBrushless);
    private RelativeEncoder rightEncoder = rightMotor.getEncoder();
    private SparkClosedLoopController rightController = rightMotor.getClosedLoopController();
    private SparkLimitSwitch limitSwitch = rightMotor.getReverseLimitSwitch();
    private SparkMaxConfig currentConfig;

    private TrapezoidProfile.Constraints profileConstraints;

    private boolean initDone = false;
    private boolean lastSwitchState = false;

    public Elevator() {
        leftMotor.setCANTimeout(Constants.kCANTimeout);
        leftMotor.setPeriodicFrameTimeout(Constants.kPeriodicFrameTimeout);

        currentConfig = new SparkMaxConfig();
        currentConfig.idleMode(IdleMode.kBrake);
        currentConfig.inverted(false);
        currentConfig.closedLoop
                .p(ElevatorConstants.kPRegular, ElevatorConstants.kRegularSlot)
                .i(ElevatorConstants.kIRegular, ElevatorConstants.kRegularSlot)
                .d(ElevatorConstants.kDRegular, ElevatorConstants.kRegularSlot);

        currentConfig.closedLoop
                .p(ElevatorConstants.kPClimb, ElevatorConstants.kClimbSlot)
                .i(ElevatorConstants.kIClimb, ElevatorConstants.kClimbSlot)
                .d(ElevatorConstants.kDClimb, ElevatorConstants.kClimbSlot);

        profileConstraints = new TrapezoidProfile.Constraints(
                ElevatorConstants.kGeneralMaxVelocity,
                ElevatorConstants.kGeneralMaxAcceleration);

        currentConfig.voltageCompensation(ElevatorConstants.kVoltageCompensation);
        currentConfig.smartCurrentLimit(ElevatorConstants.kRegularCurrentLimit);

        currentConfig.encoder.positionConversionFactor(ElevatorConstants.fPositionConversion);
        currentConfig.encoder.velocityConversionFactor(ElevatorConstants.fVelocityConversion);
        currentConfig.softLimit.forwardSoftLimit(ElevatorConstants.kSoftLimitForward).forwardSoftLimitEnabled(true);
        rightMotor.configure(currentConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        var leftConfig = new SparkMaxConfig();
        leftConfig.follow(rightMotor, true);
        leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        checkInit();
        SmartDashboard.putBoolean(getSubsystem() + ".limitSwitch", limitSwitch.isPressed());
        SmartDashboard.putNumber(getSubsystem() + ".position", rightEncoder.getPosition());
        SmartDashboard.putNumber(getSubsystem() + ".velocity", rightEncoder.getVelocity());
        SmartDashboard.putBoolean(getSubsystem() + ".initDone", initDone);
        SmartDashboard.putNumber("elevatorAppliedOutput", rightMotor.getAppliedOutput());
        SmartDashboard.putNumber("elevatorPDHRightCurrent",
                PDH.getInstance().getChannelCurrent(PDHConstants.kElevatorRightMotorChannel));
        SmartDashboard.putNumber("elevatorPDHLeftCurrent",
                PDH.getInstance().getChannelCurrent(PDHConstants.kElevatorLeftMotorChannel));
        SmartDashboard.putNumber("elevatorRightCurrent", rightMotor.getOutputCurrent());
        SmartDashboard.putNumber("elevatorLeftCurrent", leftMotor.getOutputCurrent());

    }

    public boolean isBottomSwitchActivated() {
        return limitSwitch.isPressed();
    }

    public boolean isRestrictingCoralTool() {
        return getPosition() < ElevatorConstants.kMinHeightCoralTool;
    }

    public boolean isRestrictingAlgaeTool() {
        return getPosition() < ElevatorConstants.kMinHeightAlgaeTool;
    }

    public void keepPosition() {
        rightController.setReference(getPosition(), ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                ElevatorConstants.kFeedForward);
    }

    public double getPosition() {
        return rightEncoder.getPosition();
    }

    public boolean isAtPosition(StageLevel level) {
        return isAtPosition(level.value);
    }

    public boolean isAtPosition(double position) {
        return Math.abs(getPosition() - position) < ElevatorConstants.kPositionThreshold;
    }

    private void checkInit() {
        final var switchState = limitSwitch.isPressed();
        if (switchState == true
                && lastSwitchState == false) {
            initDone = true;
            resetEncoderPosition();
        }
        lastSwitchState = switchState;
    }

    public boolean isInitDone() {
        return initDone;
    }

    public void resetEncoderPosition() {
        rightEncoder.setPosition(ElevatorConstants.kLimitSwitchPosition);
    }

    public void freezeAllMotorFunctions() {
        rightMotor.stopMotor();
    }

    public void setMotorSpeed(double speed, boolean useFeedForward) {
        rightController.setReference(speed, ControlType.kDutyCycle, ClosedLoopSlot.kSlot0,
                useFeedForward ? ElevatorConstants.kFeedForward : 0);
    }

    public void setMotorVoltage(double val, boolean useFeedForward) {
        // rightController.setReference(val, ControlType.kVoltage,
        // ClosedLoopSlot.kSlot0,
        // useFeedForward ? ElevatorConstants.kFeedForward : 0);
        rightMotor.setVoltage(val);
    }

    public Command goToPosition(StageLevel level, double maxSpeed, double maxAcceleration,
            ClosedLoopSlot closedLoopSlot) {
        return goToPosition(level.value, maxSpeed, maxAcceleration, closedLoopSlot);
    }

    public Command goToPosition(double target, double maxSpeed, double maxAcceleration, ClosedLoopSlot closedLoopSlot) {
        if (!initDone) {
            return new InstantCommand(() -> System.out.println("Not initialized. Ignoring command"));
        }

        profileConstraints = new TrapezoidProfile.Constraints(maxSpeed, maxAcceleration);

        var command = new TrapezoidProfileMovement(rightMotor, target, profileConstraints,
                () -> ElevatorConstants.kFeedForward, closedLoopSlot);
        command.addRequirements(this);
        command.schedule();
        return command;
    }

    public void setCurrentLimit(int currentLimit) {
        currentConfig.smartCurrentLimit(currentLimit);
        rightMotor.configure(currentConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public SparkMaxConfigAccessor getMotorConfig() {
        return rightMotor.configAccessor;
    }

    @Override
    public TrapezoidProfile.Constraints getProfileConstraints() {
        return profileConstraints;
    }

    @Override
    public void apply(SparkMaxConfig config, TrapezoidProfile.Constraints constraints) {
        rightMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        profileConstraints = constraints;
    }
}
