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
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.intakePos;
import frc.robot.Constants.ShooterConstants;
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

    private double kp = IntakeConstants.kp;
    private double kd = IntakeConstants.kd;
    private double ki = IntakeConstants.ki;
    private double kv = IntakeConstants.kv;
    private double oldKp = IntakeConstants.kp;
    private double oldKd = IntakeConstants.kd;
    private double oldKi = IntakeConstants.ki;
    private double oldKv = IntakeConstants.kv;

    public boolean isDown() {
        return getEncoderPosition() > IntakeConstants.kPositionThreshold;
    }

    // constructeur du sous-système
    public Intake() {
        IntakeMotor.setCANTimeout(Constants.kCANTimeout);
        IntakeMotor.setPeriodicFrameTimeout(Constants.kPeriodicFrameTimeout);

        FeedForwardConfig feedForwardConfig = new FeedForwardConfig();
        feedForwardConfig.kV(IntakeConstants.kv);

        currentConfig = new SparkMaxConfig();
        currentConfig.idleMode(IdleMode.kBrake);
        currentConfig.inverted(true);
        currentConfig.closedLoop
                .p(IntakeConstants.kp)
                .i(IntakeConstants.ki)
                .d(IntakeConstants.kd);
        currentConfig.closedLoop.apply(feedForwardConfig);

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
        
        Preferences.initDouble("intake velocity", IntakeConstants.maxVelocity);
        Preferences.initDouble("intake acceleration", IntakeConstants.maxAcceleration);
        Preferences.initDouble("intake.kP", kp);
        Preferences.initDouble("intake.kI", ki);
        Preferences.initDouble("intake.kD", kd);
        Preferences.initDouble("intake.kV", kv);

        SmartDashboard.putNumber("Intake manual voltage", 0);
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
        SmartDashboard.putNumber("Intake FeedForward", computeAf());

        oldKd = kd;
        oldKi = ki;
        oldKp = kp;
        oldKv = kv;
        kp = Preferences.getDouble("intake.kP", IntakeConstants.kp);
        ki = Preferences.getDouble("intake.kI", IntakeConstants.ki);
        kd = Preferences.getDouble("intake.kD", IntakeConstants.kd);
        kv = Preferences.getDouble("intake.kV", IntakeConstants.kv);

        if (oldKd != kd || oldKi != ki || oldKp != kp || oldKv != kv) {
            currentConfig.closedLoop
                    .p(kp)
                    .i(ki)
                    .d(kd);

            FeedForwardConfig ff = new FeedForwardConfig();
            ff.kV(kv);
            currentConfig.closedLoop.apply(ff);

            // Apply config to the motor
            IntakeMotor.configure(currentConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
    }

    public void testMotorVoltage() {
        setManualMotorVoltage(SmartDashboard.getNumber("Intake manual voltage", 0));
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

    public double computeAf() {
        return (IntakeConstants.kMaxAf
                * Math.cos(Math.toRadians(getEncoderPosition()) + Math.toRadians(IntakeConstants.kAfOffset)));
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

    /**
     * Send intake to position without using trapezoidal profile
     * 
     * @param pos
     */
    public void directToPosition(double pos) {
        intakeController.setSetpoint(pos, ControlType.kPosition, ClosedLoopSlot.kSlot0, computeAf());
    }

    public Command goToPosition(intakePos pos) {
        return goToPosition(pos.position);
    }

        
    public Command goToPosition(double target) {
         
        if (!initDone) {
            return new InstantCommand(() -> System.out.println("Not initialized. Ignoring command"));
        }

        intakeConstraints = new TrapezoidProfile.Constraints(
                Preferences.getDouble("intake velocity", IntakeConstants.maxVelocity),
                Preferences.getDouble("intake acceleration", IntakeConstants.maxAcceleration));

        var command = new TrapezoidProfileMovement(
                IntakeMotor,
                target, intakeConstraints,
                this::computeAf,
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
                computeAf());
    }

    public void freezeAllMotorFunctions() {
        IntakeMotor.stopMotor();
    }

    // controle le moteur directement avec le voltage
    public void setManualMotorVoltage(double voltage) {
        IntakeMotor.setVoltage(voltage);
    }

    // fait rouler le moteur à partir du controleur (sans kAf)
    public void setManualMotorPercentage(double percentage, boolean useKaf) {
        double feedforward = useKaf ? computeAf() : 0;
        intakeController.setSetpoint(
                percentage,
                ControlType.kDutyCycle,
                ClosedLoopSlot.kSlot0,
                feedforward);
    }

    public void setMotorSpeed(double speed) {
        intakeController.setSetpoint(
                speed,
                ControlType.kVelocity,
                ClosedLoopSlot.kSlot0, computeAf());
    }

    public void safeStop() {
        setManualMotorPercentage(0, false);
    }
}
