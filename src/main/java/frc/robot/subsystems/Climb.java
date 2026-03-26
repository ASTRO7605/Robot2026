package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.commands.TrapezoidProfileMovement;
import frc.robot.Constants.ClimbConstants.climbLvl;

public class Climb extends SubsystemBase {
    // initialisation du moteur et de l'encodeur
    private SparkMax climbMotor = new SparkMax(ClimbConstants.climbMotorId, MotorType.kBrushless);
    private RelativeEncoder climbEncoder = climbMotor.getEncoder();

    private SparkClosedLoopController climbController = climbMotor.getClosedLoopController();
    private SparkMaxConfig currentConfig;

    private boolean initDone = false;

    private TrapezoidProfile.Constraints climbConstraints;

    private double kp = ClimbConstants.kp;
    private double kd = ClimbConstants.kd;
    private double ki = ClimbConstants.ki;
    private double oldKp = ClimbConstants.kp;
    private double oldKd = ClimbConstants.kd;
    private double oldKi = ClimbConstants.ki;

    public Climb() {
        // configutation du moteur (le temp d'attente de réponse du moteur)
        climbMotor.setCANTimeout(Constants.kCANTimeout);
        climbMotor.setPeriodicFrameTimeout(Constants.kPeriodicFrameTimeout);

        currentConfig = new SparkMaxConfig();
        currentConfig.idleMode(IdleMode.kBrake);
        currentConfig.inverted(false);
        // configuration du pid
        currentConfig.closedLoop
                .p(ClimbConstants.kp)
                .i(ClimbConstants.ki)
                .d(ClimbConstants.kd);

        climbConstraints = new TrapezoidProfile.Constraints(ClimbConstants.maxVelocity, ClimbConstants.maxAcceleration);

        // limitation du courant et de la tension pour protéger le moteur et la batterie
        currentConfig.voltageCompensation(Constants.kVoltageCompensation);
        currentConfig.smartCurrentLimit(ClimbConstants.kCurrentLimit);

        // Conversion des unités de l'encodeur
        currentConfig.encoder.positionConversionFactor(ClimbConstants.fPositionConversion);
        currentConfig.encoder.velocityConversionFactor(ClimbConstants.fVelocityConversion);

        // limitation du mouvement du moteur pour éviter les dommages mécaniques
        currentConfig.softLimit.forwardSoftLimit(ClimbConstants.kSoftLimitForward).forwardSoftLimitEnabled(true);

        climbMotor.configure(currentConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        climbEncoder.setPosition(0);
        initDone = false;

        Preferences.initDouble("climb.kP", kp);
        Preferences.initDouble("climb.kI", ki);
        Preferences.initDouble("climb.kD", kd);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(getSubsystem() + ".position", climbEncoder.getPosition());
        SmartDashboard.putNumber(getSubsystem() + ".velocity", climbEncoder.getVelocity());
        SmartDashboard.putBoolean(getSubsystem() + ".initDone", initDone);
        SmartDashboard.putNumber("climbAppliedOutput", climbMotor.getAppliedOutput());
        SmartDashboard.putNumber("climbRightCurrent", climbMotor.getOutputCurrent());
        oldKd = kd;
        oldKi = ki;
        oldKp = kp;
        kp = Preferences.getDouble("climb.kP", ClimbConstants.kp);
        ki = Preferences.getDouble("climb.kI", ClimbConstants.ki);
        kd = Preferences.getDouble("climb.kD", ClimbConstants.kd);

        if (oldKd != kd || oldKi != ki || oldKp != kp) {
            currentConfig.closedLoop
                    .p(kp)
                    .i(ki)
                    .d(kd);

            // Apply config to the motor
            climbMotor.configure(currentConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }

    }

    // fait rouler le moteur à partir du controleur
    public void setMotorPercentage(double percentage, boolean useFeedForward) {
        climbController.setSetpoint(
                percentage,
                ControlType.kDutyCycle,
                ClosedLoopSlot.kSlot0,
                useFeedForward ? ClimbConstants.feedforwards : 0);
    }

    // controle le moteur directement avec le voltage
    public void setMotorVoltage(double voltage) {
        climbMotor.setVoltage(voltage);
    }

    /**
     * Send climb to position without using trapezoidal profile
     * 
     * @param pos
     */
    public void directToPosition(double pos) {
        climbController.setSetpoint(pos, ControlType.kPosition, ClosedLoopSlot.kSlot0, ClimbConstants.feedforwards);
    }

    /**
     * 
     * @param lvl             la position désirée du Climb
     * @param maxSpeed        la vitesse maximale du mouvement
     * @param maxAcceleration l'accélération maximale du mouvement
     * @return la commande qui exécute ce mouvement
     */
    public Command goToPosition(climbLvl lvl, double maxSpeed, double maxAcceleration) {
        return goToPosition(lvl.position, maxSpeed, maxAcceleration, ClosedLoopSlot.kSlot0);
    }

    /**
     * Fait le Climb aller à la position désirée en utilisant le mouvement
     * trapézoïdal
     * afin de ne pas avoir d'accélération ou décélération trop brusque.
     * 
     * @param target          la position désirée en inches
     * @param maxSpeed        la vitesse maximale du mouvement
     * @param maxAcceleration l'accélération maximale du mouvement
     * @param closedLoopSlot  le slot de contrôle à utiliser pour le PID
     *                        (généralement kSlot0)
     * @return la commande qui exécute ce mouvement
     */
    public Command goToPosition(double target, double maxSpeed, double maxAcceleration, ClosedLoopSlot closedLoopSlot) {
        if (!initDone) {
            return new InstantCommand(() -> System.out.println("Not initialized. Ignoring command"));
        }

        climbConstraints = new TrapezoidProfile.Constraints(maxSpeed, maxAcceleration);

        var command = new TrapezoidProfileMovement(climbMotor, target, climbConstraints,
                () -> ClimbConstants.feedforwards, closedLoopSlot);
        command.addRequirements(this);
        CommandScheduler.getInstance().schedule(command);
        return command;
    }

    /**
     * Maintient la position actuelle du Climb
     */
    public void keepPosition() {
        climbController.setSetpoint(
                getPosition(),
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                ClimbConstants.feedforwards);
    }

    /**
     * Retourne la position actuelle du Climb
     * 
     * @return
     */
    public double getPosition() {
        return climbEncoder.getPosition();
    }

    public boolean isAtPosition(climbLvl level) {
        return isAtPosition(level.position);
    }

    public boolean isAtPosition(double position) {
        return Math.abs(getPosition() - position) < ClimbConstants.kPositionThreshold;
    }

    public boolean isInitDone() {
        return initDone;
    }

    public void resetEncoderPosition() {
        climbEncoder.setPosition(ClimbConstants.kInitPosition);
    }

    public void setInitDone() {
        initDone = true;
    }

    public void freezeAllMotorFunctions() {
        climbMotor.stopMotor();
    }

    public void safeStop() {
        setMotorPercentage(0, false);
    }

    public boolean isMotorStopped() {
        return Math.abs(climbEncoder.getVelocity()) <= ClimbConstants.kStoppedMotorThreshold;
    }
}
