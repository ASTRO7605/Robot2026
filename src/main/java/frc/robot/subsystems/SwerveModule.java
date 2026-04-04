package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;

public class SwerveModule {
    public int m_moduleNumber;
    private final Rotation2d m_angleOffset;

    private final TalonFX m_angleMotor;
    private final TalonFX m_driveMotor;
    private final CANcoder m_angleEncoder;

    private final SimpleMotorFeedforward m_driveFeedForward = new SimpleMotorFeedforward(
            Constants.DriveConstants.driveKS, Constants.DriveConstants.driveKV, Constants.DriveConstants.driveKA);

    /* drive motor control requests */
    private final DutyCycleOut m_driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage m_driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PositionVoltage m_anglePosition = new PositionVoltage(0);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.m_moduleNumber = moduleNumber;
        this.m_angleOffset = moduleConstants.angleOffset;

        final var canbus = new CANBus(DriveConstants.kCanivoreBusId);
        /* Angle Encoder Config */

        m_angleEncoder = new CANcoder(moduleConstants.cancoderID, canbus);
        m_angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);

        /* Angle Motor Config */
        m_angleMotor = new TalonFX(moduleConstants.angleMotorID, canbus);
        m_angleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
        resetToAbsolute();

        /* Drive Motor Config */
        m_driveMotor = new TalonFX(moduleConstants.driveMotorID, canbus);
        m_driveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        m_driveMotor.getConfigurator().setPosition(0.0);

        setNeutralMode(NeutralModeValue.Brake);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState.optimize(getState().angle);
        m_angleMotor.setControl(m_anglePosition.withPosition(desiredState.angle.getRotations()));
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            m_driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.DriveConstants.kMaxTeleopSpeed;
            m_driveMotor.setControl(m_driveDutyCycle);
        } else {
            m_driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond,
                    Constants.DriveConstants.wheelCircumference);
            m_driveVelocity.FeedForward = m_driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            m_driveMotor.setControl(m_driveVelocity);
        }
    }

    public Rotation2d getCANcoder() {
        return Rotation2d.fromRotations(m_angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public void resetToAbsolute() {
        double absolutePosition = getCANcoder().getRotations() - m_angleOffset.getRotations();
        m_angleMotor.setPosition(absolutePosition);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                Conversions.RPSToMPS(m_driveMotor.getVelocity().getValueAsDouble(),
                        Constants.DriveConstants.wheelCircumference),
                Rotation2d.fromRotations(m_angleMotor.getPosition().getValueAsDouble()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                Conversions.rotationsToMeters(m_driveMotor.getPosition().getValueAsDouble(),
                        Constants.DriveConstants.wheelCircumference),
                Rotation2d.fromRotations(m_angleMotor.getPosition().getValueAsDouble()));
    }

    public void setNeutralMode(NeutralModeValue neutralMode) {
        m_driveMotor.setNeutralMode(neutralMode);
        m_angleMotor.setNeutralMode(neutralMode);
    }

    public void sysIdMotorVoltage(double voltage) {
        m_driveMotor.setVoltage(voltage);
    }

    public Voltage getAppliedVoltage() {
        return m_driveMotor.getMotorVoltage().getValue();
    }
}