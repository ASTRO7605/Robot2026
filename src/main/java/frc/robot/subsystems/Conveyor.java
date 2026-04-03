package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utils.PDH;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Conveyor extends SubsystemBase {

    private double kp = ConveyorConstants.kp;
    private double kd = ConveyorConstants.kd;
    private double ki = ConveyorConstants.ki;
    private double kv = ConveyorConstants.kv;
    private double oldKp = ConveyorConstants.kp;
    private double oldKd = ConveyorConstants.kd;
    private double oldKi = ConveyorConstants.ki;
    private double oldKv = ConveyorConstants.kv;
    // initialisation du moteur et de l'encodeur
    private SparkMax ConvMotor = new SparkMax(ConveyorConstants.conveyorMotorId, MotorType.kBrushless);

    // moteur pour le leftIntake (celui du bas)
    private SparkMax intakeRollerMotor = new SparkMax(ConveyorConstants.intakeRollerMotorId, MotorType.kBrushless);
    private RelativeEncoder convEncoder = ConvMotor.getEncoder();
    private RelativeEncoder intakeRollerEncoder = intakeRollerMotor.getEncoder();
    private SparkMaxConfig conveyorConfig;
    private SparkMaxConfig intakeRollerConfig;
    private SparkClosedLoopController intakeRollerController = intakeRollerMotor.getClosedLoopController();
    private SparkClosedLoopController convController = ConvMotor.getClosedLoopController();

    // constructeur du sous-système
    public Conveyor() {
        // configutation du moteur (le temp d'attente de réonse du moteur)
        ConvMotor.setCANTimeout(Constants.kCANTimeout);
        ConvMotor.setPeriodicFrameTimeout(Constants.kPeriodicFrameTimeout);

        intakeRollerMotor.setCANTimeout(Constants.kCANTimeout);
        intakeRollerMotor.setPeriodicFrameTimeout(Constants.kPeriodicFrameTimeout);

        FeedForwardConfig feedForwardConfig = new FeedForwardConfig();
        feedForwardConfig.kV(ShooterConstants.kv);

        conveyorConfig = new SparkMaxConfig();
        conveyorConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        conveyorConfig.inverted(true);

        conveyorConfig.encoder.positionConversionFactor(ConveyorConstants.fPositionConversion);
        conveyorConfig.encoder.velocityConversionFactor(ConveyorConstants.fVelocityConversion);

        intakeRollerConfig = new SparkMaxConfig();
        intakeRollerConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        intakeRollerConfig.inverted(false);

        intakeRollerConfig.encoder.positionConversionFactor(ConveyorConstants.fPositionConversion);
        intakeRollerConfig.encoder.velocityConversionFactor(ConveyorConstants.fVelocityConversion);

        conveyorConfig.closedLoop
                .p(ConveyorConstants.kp)
                .i(ConveyorConstants.ki)
                .d(ConveyorConstants.kd);
        conveyorConfig.closedLoop.apply(feedForwardConfig);

        intakeRollerConfig.closedLoop
                .p(ConveyorConstants.kp)
                .i(ConveyorConstants.ki)
                .d(ConveyorConstants.kd);
        intakeRollerConfig.closedLoop.apply(feedForwardConfig);

        Preferences.initDouble("Conveyor.kP", kp);
        Preferences.initDouble("Conveyor.kI", ki);
        Preferences.initDouble("Conveyor.kD", kd);
        Preferences.initDouble("Conveyor.kV", kv);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(getSubsystem() + ".ConveyorVelocity", convEncoder.getVelocity());
        SmartDashboard.putNumber(getSubsystem() + ".IntakeVelocity", intakeRollerEncoder.getVelocity());

        oldKd = kd;
        oldKi = ki;
        oldKp = kp;
        oldKv = kv;
        kp = Preferences.getDouble("Conveyor.kP", ConveyorConstants.kp);
        ki = Preferences.getDouble("Conveyor.kI", ConveyorConstants.ki);
        kd = Preferences.getDouble("Conveyor.kD", ConveyorConstants.kd);
        kv = Preferences.getDouble("Conveyor.kV", ConveyorConstants.kv);

        if (oldKd != kd || oldKi != ki || oldKp != kp || oldKv != kv) {
            intakeRollerConfig.closedLoop
                    .p(kp)
                    .i(ki)
                    .d(kd);
            conveyorConfig.closedLoop.p(kp)
                    .i(ki)
                    .d(kd);

            FeedForwardConfig ff = new FeedForwardConfig();
            ff.kV(kv);
            intakeRollerConfig.closedLoop.apply(ff);
            conveyorConfig.closedLoop.apply(ff);

            ConvMotor.configure(conveyorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            intakeRollerMotor.configure(intakeRollerConfig, ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters);
        }
        SmartDashboard.putNumber("ConveyorVelocity", convEncoder.getVelocity());
        SmartDashboard.putBoolean("ConveyorStopped", isConveyorStopped());
        SmartDashboard.putNumber("intakeWheelVelocity", intakeRollerEncoder.getVelocity());
        SmartDashboard.putBoolean("IntakeStopped", isIntakeWheelStopped());
        SmartDashboard.putNumber("IntakeOutput", intakeRollerMotor.getAppliedOutput());
        SmartDashboard.putNumber("Intake Roller Motor Current", intakeRollerMotor.getOutputCurrent());
        // SmartDashboard.putNumber("Intake Roller PDH Current",
        // PDH.getInstance().getChannelCurrent(ConveyorConstants.intakeRollerPdhChannel));

    }

    // fait tourner les roues du intake pour faire entrer les balles
    // *********************************************A FIX
    public void setConveyorWheelsSpeed(double speed) {
        intakeRollerController.setSetpoint(
                speed,
                ControlType.kVelocity,
                ClosedLoopSlot.kSlot0);
        convController.setSetpoint(
                speed,
                ControlType.kVelocity,
                ClosedLoopSlot.kSlot0);

    }

    // fait tourner les roues du intake pour faire entrer les balles
    public void conveyorWheelsIn() {
        setConveyorOutputPercentage(ConveyorConstants.manualPercentageIn);
    }

    // fait tourner les roues du convoyeur pour faire sortir les balles
    public void conveyorWheelsOut() {
        setConveyorOutputPercentage(ConveyorConstants.manualPercentageOut);
    }

    public void setConveyorOutputPercentage(double percentage) {
        intakeRollerController.setSetpoint(
                -percentage,
                ControlType.kDutyCycle,
                ClosedLoopSlot.kSlot0);
        convController.setSetpoint(
                percentage,
                ControlType.kDutyCycle,
                ClosedLoopSlot.kSlot0);
    }

    // arrête les roues du convoyeur
    public void ConveyorWheelOff() {
        ConvMotor.stopMotor();
        intakeRollerMotor.stopMotor();
    }

    public boolean isConveyorStopped() {
        return Math.abs(convEncoder.getVelocity()) < ConveyorConstants.kThresholdMotorStopped;
    }

    public boolean isIntakeWheelStopped() {
        return Math.abs(intakeRollerEncoder.getVelocity()) < ConveyorConstants.kThresholdMotorStopped;
    }

}
