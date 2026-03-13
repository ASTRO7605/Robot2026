package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.utils.SparkMaxTunableMotorSystem;


public class Intake extends SubsystemBase {
    private SparkMax IntakeMotor = new SparkMax(IntakeConstants.rightIntakeMotorId, MotorType.kBrushless);
    private RelativeEncoder intakeEncoder = IntakeMotor.getEncoder();

    private SparkClosedLoopController intakeController = IntakeMotor.getClosedLoopController();
    private SparkLimitSwitch limitSwitch = IntakeMotor.getReverseLimitSwitch();
    private SparkMaxConfig currentConfig;

    private boolean initDone = false;
    private boolean lastSwitchState = false;

    private TrapezoidProfile.Constraints intakeConstraints;

    //constructeur du sous-système
    public Intake(){
        IntakeMotor.setCANTimeout(Constants.kCANTimeout);
        IntakeMotor.setPeriodicFrameTimeout(Constants.kPeriodicFrameTimeout);

        currentConfig = new SparkMaxConfig();
        currentConfig.idleMode(IdleMode.kBrake);
        currentConfig.inverted(false);
        currentConfig.closedLoop
                .p(IntakeConstants.kp)
                .i(IntakeConstants.ki)
                .d(IntakeConstants.kd);

        currentConfig.voltageCompensation(IntakeConstants.kVoltageCompensation);
        currentConfig.smartCurrentLimit(IntakeConstants.kCurrentLimit);

        currentConfig.encoder.positionConversionFactor(IntakeConstants.fPositionConversion);
        currentConfig.encoder.velocityConversionFactor(IntakeConstants.fVelocityConversion);
        currentConfig.softLimit.forwardSoftLimit(IntakeConstants.kSoftLimitForward).forwardSoftLimitEnabled(true);

        IntakeMotor.configure(currentConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        intakeConstraints = new TrapezoidProfile.Constraints(
                IntakeConstants.maxVelocity,
                IntakeConstants.maxAcceleration);
    }

    @Override
    public void periodic() {
        checkInit();
        SmartDashboard.putBoolean(getSubsystem() + ".limitSwitch", limitSwitch.isPressed());
        SmartDashboard.putNumber(getSubsystem() + ".position", intakeEncoder.getPosition());
        SmartDashboard.putNumber(getSubsystem() + ".velocity", intakeEncoder.getVelocity());
        SmartDashboard.putBoolean(getSubsystem() + ".initDone", initDone);
        SmartDashboard.putNumber("intakeAppliedOutput", IntakeMotor.getAppliedOutput());
        SmartDashboard.putNumber("intakeRightCurrent", IntakeMotor.getOutputCurrent());
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
        return (IntakeConstants.kMaxFF * Math.cos(Math.toRadians(getPosition())));
    }

    public void resetEncoderPosition() {
        intakeEncoder.setPosition(IntakeConstants.kLimitSwitchPosition);
    }
    
    public double getPosition() {
        return intakeEncoder.getPosition();
    }

}
