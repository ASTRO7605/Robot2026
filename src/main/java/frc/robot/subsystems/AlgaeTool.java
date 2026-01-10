package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkMaxConfigAccessor;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeToolConstants;
import frc.robot.Constants.AlgaeToolConstants.AlgaeToolPosition;
import frc.robot.utils.SparkMaxTunableMotorSystem;

public class AlgaeTool extends SubsystemBase implements SparkMaxTunableMotorSystem {
    private SparkMax motor = new SparkMax(AlgaeToolConstants.kMotorId, MotorType.kBrushless);
    private AbsoluteEncoder encoder = motor.getAbsoluteEncoder();
    private SparkClosedLoopController controller = motor.getClosedLoopController();

    private boolean isElevatorRestrictionActive = true;

    public AlgaeTool() {
        motor.setCANTimeout(Constants.kCANTimeout);
        motor.setPeriodicFrameTimeout(Constants.kPeriodicFrameTimeout);

        var config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.inverted(true);
        config.closedLoop
                .p(AlgaeToolConstants.kPFlywheel)
                .i(AlgaeToolConstants.kIFlywheel)
                .d(AlgaeToolConstants.kDFlywheel);

        config.softLimit.reverseSoftLimit(AlgaeToolConstants.kSoftLimitReverse).reverseSoftLimitEnabled(true);
        config.softLimit.forwardSoftLimit(AlgaeToolConstants.kSoftLimitForward).forwardSoftLimitEnabled(true);

        config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        config.voltageCompensation(AlgaeToolConstants.kVoltageCompensation);
        config.smartCurrentLimit(AlgaeToolConstants.kCurrentLimit);

        config.absoluteEncoder.setSparkMaxDataPortConfig().positionConversionFactor(180).zeroOffset(0.55);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setElevatorRestriction(boolean state) {
        isElevatorRestrictionActive = state;
    }

    public void goToPosition(AlgaeToolPosition p) {
        controller.setReference(p.value, ControlType.kPosition);
    }

    public void keepPosition() {
        controller.setReference(getPosition(), ControlType.kPosition);
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    public boolean isAtPosition(AlgaeToolPosition p) {
        return Math.abs(encoder.getPosition() - p.value) < AlgaeToolConstants.kPositionThreshold;
    }

    public void setMotorSpeed(double speed) {
        motor.set(speed);
    }

    public void setMotorVoltage(double val) {
        motor.setVoltage(val);
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
        return null;
    }

    @Override
    public void apply(SparkMaxConfig config, TrapezoidProfile.Constraints constraints) {
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("algaeToolPosition", encoder.getPosition());
        SmartDashboard.putNumber("algaeToolVelocity", encoder.getVelocity());
    }
}
