package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
    private SparkLimitSwitch limitSwitch = climbMotor.getReverseLimitSwitch();
    private SparkMaxConfig currentConfig;

    private boolean initDone = false;
    private boolean lastSwitchState = false;

    private TrapezoidProfile.Constraints climbConstraints;

    public Climb() {
        checkInit();
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
        currentConfig.voltageCompensation(ClimbConstants.kVoltageCompensation);
        currentConfig.smartCurrentLimit(ClimbConstants.kCurrentLimit);

        // Conversion des unités de l'encodeur
        currentConfig.encoder.positionConversionFactor(ClimbConstants.fPositionConversion);
        currentConfig.encoder.velocityConversionFactor(ClimbConstants.fVelocityConversion);

        // limitation du mouvement du moteur pour éviter les dommages mécaniques
        currentConfig.softLimit.forwardSoftLimit(ClimbConstants.kSoftLimitForward).forwardSoftLimitEnabled(true);

        climbMotor.configure(currentConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean(getSubsystem() + ".limitSwitch", limitSwitch.isPressed());
        SmartDashboard.putNumber(getSubsystem() + ".position", climbEncoder.getPosition());
        SmartDashboard.putNumber(getSubsystem() + ".velocity", climbEncoder.getVelocity());
        SmartDashboard.putBoolean(getSubsystem() + ".initDone", initDone);
        SmartDashboard.putNumber("elevatorAppliedOutput", climbMotor.getAppliedOutput());
        SmartDashboard.putNumber("elevatorRightCurrent", climbMotor.getOutputCurrent());

    }

    // fait rouler le moteur à partir du controleur
    public void setMotorSpeed(double speed, boolean useFeedForward) {
        climbController.setSetpoint(
                speed, 
                ControlType.kDutyCycle,
                ClosedLoopSlot.kSlot0,
                useFeedForward ? ClimbConstants.feedforwards : 0);
    }

    // controle le moteur directement avec le voltage
    public void setMotorVoltage(double voltage) {
        climbMotor.setVoltage(voltage);
    }

    /**
     * 
     * @param lvl la position désirée du Climb (stowed, Lvl1, Lvl2, Lvl3)
     * @param maxSpeed la vitesse maximale du mouvement
     * @param maxAcceleration l'accélération maximale du mouvement
     * @param closedLoopSlot le slot de contrôle à utiliser pour le PID (généralement kSlot0)
     * @return la commande qui exécute ce mouvement
     */
    public Command goToPosition(climbLvl lvl, double maxSpeed, double maxAcceleration,
            ClosedLoopSlot closedLoopSlot) {
        return goToPosition(lvl.position, maxSpeed, maxAcceleration, closedLoopSlot);
    }

    /**
     * Fait le Climb aller à la position désirée en utilisant le mouvement trapézoïdal 
     * afin de ne pas avoir d'accélération ou décélération trop brusque.
     * @param target la position désirée en inches
     * @param maxSpeed la vitesse maximale du mouvement
     * @param maxAcceleration l'accélération maximale du mouvement
     * @param closedLoopSlot le slot de contrôle à utiliser pour le PID (généralement kSlot0)
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

    /**
     * Vérifie si le Climb a atteint la position de départ en utilisant le limit switch.
     * Si le switch est activé alors que le switch n'était pas activé lors du dernier check, 
     * alors on considère que l'initialisation est terminée et on reset la position de l'encodeur.
     */
    private void checkInit() {
        final var switchState = limitSwitch.isPressed();
        if (switchState && !lastSwitchState) {
            initDone = true;
            resetEncoderPosition();
        }
        lastSwitchState = switchState;
    }

    public boolean isInitDone() {
        return initDone;
    }

    public void resetEncoderPosition() {
        climbEncoder.setPosition(ClimbConstants.kLimitSwitchPosition);
    }

    public void freezeAllMotorFunctions() {
        climbMotor.stopMotor();
    }

    /**
     * Met à jour la limite de courant du moteur pour protéger le moteur et la batterie.
     * @param currentLimit
     */
    public void setCurrentLimit(int currentLimit) {
        currentConfig.smartCurrentLimit(currentLimit);
        climbMotor.configure(currentConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public boolean isBottomSwitchActivated() {
        return limitSwitch.isPressed();
    }
}
