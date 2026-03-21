package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.intakePos;
import frc.robot.commands.TrapezoidProfileMovement;

public class Intake extends SubsystemBase {
    private SparkMax IntakeMotor = new SparkMax(IntakeConstants.rightIntakeMotorId, MotorType.kBrushless);
    private RelativeEncoder intakeEncoder = IntakeMotor.getEncoder();

    private SparkClosedLoopController intakeController = IntakeMotor.getClosedLoopController();
    private SparkLimitSwitch limitSwitch = IntakeMotor.getForwardLimitSwitch();
    private SparkMaxConfig currentConfig;

    private boolean initDone = false;
    private boolean lastSwitchState = false;

    private TrapezoidProfile.Constraints intakeConstraints;

    // constructeur du sous-système
    public Intake() {
        IntakeMotor.setCANTimeout(Constants.kCANTimeout);
        IntakeMotor.setPeriodicFrameTimeout(Constants.kPeriodicFrameTimeout);

        currentConfig = new SparkMaxConfig();
        currentConfig.idleMode(IdleMode.kCoast);
        currentConfig.inverted(true);
        currentConfig.closedLoop
                .p(IntakeConstants.kp)
                .i(IntakeConstants.ki)
                .d(IntakeConstants.kd);

        currentConfig.voltageCompensation(Constants.kVoltageCompensation);
        currentConfig.smartCurrentLimit(IntakeConstants.kCurrentLimit);

        currentConfig.encoder.positionConversionFactor(IntakeConstants.fPositionConversion);
        currentConfig.encoder.velocityConversionFactor(IntakeConstants.fVelocityConversion);
        currentConfig.softLimit.reverseSoftLimit(IntakeConstants.kSoftLimitReverse).reverseSoftLimitEnabled(true);

        IntakeMotor.configure(currentConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        intakeConstraints = new TrapezoidProfile.Constraints(
                IntakeConstants.maxVelocity,
                IntakeConstants.maxAcceleration);

        initDone = false;
    }

    @Override
    public void periodic() {
        checkInit();
        SmartDashboard.putBoolean(getSubsystem() + ".limitSwitch", limitSwitch.isPressed());
        SmartDashboard.putNumber(getSubsystem() + ".encoderPosition", getEncoderPosition());
        SmartDashboard.putNumber(getSubsystem() + ".velocity", intakeEncoder.getVelocity());
        SmartDashboard.putBoolean(getSubsystem() + ".initDone", initDone);
        SmartDashboard.putNumber("intakeAppliedOutput", IntakeMotor.getAppliedOutput());
        SmartDashboard.putNumber("intakeRightCurrent", IntakeMotor.getOutputCurrent());
        SmartDashboard.putNumber("intake.kP", IntakeConstants.kp);
        SmartDashboard.putNumber("intake.kI", IntakeConstants.ki);
        SmartDashboard.putNumber("intake.kD", IntakeConstants.kd);
        SmartDashboard.putNumber("Intake FeedForward", computeFF());
    }

    private void checkInit() {
        final var switchState = limitSwitch.isPressed();
        if (switchState == true && lastSwitchState == false) {
            initDone = true;
            resetEncoderPosition();
        }
        lastSwitchState = switchState;
    }

    public boolean isInitDone() {
        return initDone;
    }

    public void setInitAsDone() {
        initDone = true;
    }

    public double computeFF() {
        return (IntakeConstants.kMaxFF * Math.cos(Math.toRadians(getEncoderPosition())));
    }

    public void resetEncoderPosition() {
        intakeEncoder.setPosition(IntakeConstants.kLimitSwitchPosition);
    }

    public double getEncoderPosition() {
        return intakeEncoder.getPosition();
    }

    public boolean isLimitSwitchActivated() {
        return limitSwitch.isPressed();
    }

    public Command goToPosition(intakePos pos) {
        return goToPosition(pos.position);
    }

    private Command goToPosition(double target) {
        if (!initDone) {
            return new InstantCommand(() -> System.out.println("Not initialized. Ignoring command"));
        }

        var command = new TrapezoidProfileMovement(
                IntakeMotor,
                target, intakeConstraints,
                this::computeFF,
                ClosedLoopSlot.kSlot0);

        command.addRequirements(this);
        CommandScheduler.getInstance().schedule(command);
        return command;
    }

    /**
     * Maintient la position actuelle de l'intake avec l'encoder
     */
    public void keepPosition() {
        intakeController.setSetpoint(
                getEncoderPosition(),
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                computeFF());
    }

    public void freezeAllMotorFunctions() {
        IntakeMotor.stopMotor();
    }

    // controle le moteur directement avec le voltage
    public void setManualMotorVoltage(double voltage) {
        IntakeMotor.setVoltage(voltage);
    }

    // fait rouler le moteur à partir du controleur (sans kAf)
    public void setManualMotorPercentage(double percentage) {
        intakeController.setSetpoint(
                percentage,
                ControlType.kDutyCycle,
                ClosedLoopSlot.kSlot0);
    }
    public void setManualMotorSpeed(double speed) {
        intakeController.setSetpoint(
                speed,
                ControlType.kVelocity,
                ClosedLoopSlot.kSlot0);
    }

    public void safeStop() {
        setManualMotorPercentage(0);
    }
}
