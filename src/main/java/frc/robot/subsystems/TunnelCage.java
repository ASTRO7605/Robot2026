package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.TunnelCageConstants;
import frc.robot.commands.TrapezoidProfileMovement;

public class TunnelCage extends SubsystemBase {
    private SparkMax motor = new SparkMax(TunnelCageConstants.kMotorId, MotorType.kBrushless);
    private RelativeEncoder encoder = motor.getEncoder();
    private SparkClosedLoopController controller = motor.getClosedLoopController();

    private TrapezoidProfile.Constraints profileConstraints;

    private Servo servo;
    private DigitalInput cageSensor;

    private boolean initDone = false;
    private double currentServoPosition;

    public TunnelCage() {
        servo = new Servo(TunnelCageConstants.kServoPwmPort);
        servo.setBoundsMicroseconds(TunnelCageConstants.kMaxPulseServo, 0,
                (TunnelCageConstants.kMaxPulseServo + TunnelCageConstants.kMinPulseServo) / 2, 0,
                TunnelCageConstants.kMinPulseServo);

        cageSensor = new DigitalInput(TunnelCageConstants.kCageSensorID);

        // motor config
        motor.setCANTimeout(Constants.kCANTimeout);
        motor.setPeriodicFrameTimeout(Constants.kPeriodicFrameTimeout);

        var config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast);
        config.inverted(true);
        config.closedLoop
                .p(TunnelCageConstants.kPFlywheel)
                .i(TunnelCageConstants.kIFlywheel)
                .d(TunnelCageConstants.kDFlywheel);

        config.voltageCompensation(TunnelCageConstants.kVoltageCompensation);
        config.smartCurrentLimit(TunnelCageConstants.kCurrentLimit);
        config.encoder.positionConversionFactor(TunnelCageConstants.fConversionPositionMeters);
        config.encoder.velocityConversionFactor(TunnelCageConstants.fConversionVelocityMeters);


        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        profileConstraints = new TrapezoidProfile.Constraints(TunnelCageConstants.kMoveCageSpeed,
                TunnelCageConstants.kMoveCageAcceleration);

        currentServoPosition = TunnelCageConstants.kRetractedServoPosition;
        resetEncoderPosition(0);
    }

    public void keepPosition() {
        controller.setReference(getPosition(), ControlType.kPosition);
    }

    public Command goToPositionTrapezoid(double target) {
        if (!initDone) {
            return new InstantCommand(() -> System.out.println("Not initialized. Ignoring command"));
        }

        var command = new TrapezoidProfileMovement(motor, target, profileConstraints,
                () -> 0.0, ClosedLoopSlot.kSlot0);
        command.addRequirements(this);
        command.schedule();
        return command;
    }

    public boolean isCageInTunnel() {
        return !cageSensor.get();
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    public void setMotorSpeed(double speed) {
        motor.set(speed);
    }

    public boolean isInitDone() {
        return initDone;
    }

    public void setInitAsDone() {
        initDone = true;
    }

    public void resetEncoderPosition(double position) {
        encoder.setPosition(position);
    }

    public boolean isMotorStopped() {
        return (Math.abs(encoder.getVelocity()) <= TunnelCageConstants.kThresholdMotorStopped);
    }

    public void goToPosition(double position) {
        controller.setReference(position, ControlType.kPosition);
    }

    public boolean isAtPosition(double position) {
        return Math.abs(encoder.getPosition() - position) <= TunnelCageConstants.kPositionThreshold;
    }

    public void freezeAllMotorFunctions() {
        motor.stopMotor();
    }

    public void setServoPosition(double position) {
        currentServoPosition = position;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("TunnelCagePosition", getPosition());

        SmartDashboard.putBoolean("IsCageInTunnel", isCageInTunnel());

        servo.set(currentServoPosition);
    }
}
