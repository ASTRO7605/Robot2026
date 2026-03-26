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
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.commands.TrapezoidProfileMovement;
import frc.robot.utils.ShotCalculator;

public class Tourelle extends SubsystemBase {
    // initialisation du moteur et de l'encodeur
    private SparkMax turretMotor = new SparkMax(TurretConstants.turretMotorId, MotorType.kBrushless);
    private RelativeEncoder turretEncoder = turretMotor.getEncoder();

    private SparkClosedLoopController turretController = turretMotor.getClosedLoopController();
    private SparkLimitSwitch limitSwitch = turretMotor.getReverseLimitSwitch();
    private SparkMaxConfig currentConfig;

    private boolean initDone = false;
    private boolean lastSwitchState = false;

    private TrapezoidProfile.Constraints turretConstraints;

    private double kp = ClimbConstants.kp;
    private double kd = ClimbConstants.kd;
    private double ki = ClimbConstants.ki;
    private double oldKp = ClimbConstants.kp;
    private double oldKd = ClimbConstants.kd;
    private double oldKi = ClimbConstants.ki;

    // constructeur du sous-système
    public Tourelle() {
        // configutation du moteur (le temp d'attente de réponse du moteur)
        turretMotor.setCANTimeout(Constants.kCANTimeout);
        turretMotor.setPeriodicFrameTimeout(Constants.kPeriodicFrameTimeout);

        currentConfig = new SparkMaxConfig();
        currentConfig.idleMode(IdleMode.kBrake);
        currentConfig.inverted(true);
        // configuration du pid
        currentConfig.closedLoop
                .p(TurretConstants.kp)
                .i(TurretConstants.ki)
                .d(TurretConstants.kd);

        turretConstraints = new TrapezoidProfile.Constraints(TurretConstants.maxVelocity,
                TurretConstants.maxAcceleration);

        // limitation du courant et de la tension pour protéger le moteur et la batterie
        currentConfig.voltageCompensation(Constants.kVoltageCompensation);
        currentConfig.smartCurrentLimit(TurretConstants.kCurrentLimit);

        // Conversion des unités de l'encodeur
        currentConfig.encoder.positionConversionFactor(TurretConstants.fPositionConversion);
        currentConfig.encoder.velocityConversionFactor(TurretConstants.fVelocityConversion);

        // limitation du mouvement du moteur pour éviter les dommages mécaniques
        currentConfig.softLimit.forwardSoftLimit(TurretConstants.kSoftLimitForward).forwardSoftLimitEnabled(true);

        turretMotor.configure(currentConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        Preferences.initDouble("climb.kP", kp);
        Preferences.initDouble("climb.kI", ki);
        Preferences.initDouble("climb.kD", kd);
    }

    @Override
    public void periodic() {
        checkInit();
        SmartDashboard.putBoolean(getSubsystem() + ".limitSwitch", limitSwitch.isPressed());
        SmartDashboard.putNumber(getSubsystem() + ".position", turretEncoder.getPosition());
        SmartDashboard.putNumber(getSubsystem() + ".velocity", turretEncoder.getVelocity());
        SmartDashboard.putBoolean(getSubsystem() + ".initDone", initDone);

        oldKd = kd;
        oldKi = ki;
        oldKp = kp;
        kp = Preferences.getDouble("tourelle.kP", TurretConstants.kp);
        ki = Preferences.getDouble("tourelle.kI", TurretConstants.ki);
        kd = Preferences.getDouble("tourelle.kD", TurretConstants.kd);

        if (oldKd != kd || oldKi != ki || oldKp != kp) {
            currentConfig.closedLoop
                    .p(kp)
                    .i(ki)
                    .d(kd);

            // Apply config to the motor
            turretMotor.configure(currentConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
    }

    /**
     * Vérifie si la tourelle a atteint la position de départ en utilisant le limit
     * switch.
     * Si le switch est activé alors que le switch n'était pas activé lors du
     * dernier check, alors on considère que l'initialisation est terminée et on
     * reset la position de l'encodeur.
     */
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

    public double getPosition() {
        return turretEncoder.getPosition();
    }

    public void setMotorSpeed(double speed) {
        turretController.setSetpoint(speed, ControlType.kDutyCycle, ClosedLoopSlot.kSlot0, 0);
    }

    public boolean isLimitSwitchActivated() {
        return limitSwitch.isPressed();
    }

    public void resetEncoderPosition() {
        turretEncoder.setPosition(TurretConstants.kLimitSwitchPosition);
    }

    /**
     * Déplace la tourelle vers la cible (AprilTag) en utilisant
     * le mouvement trapézoidal afin de ne pas avoir d'accélération ou décélération
     * trop brusque.
     * MÉTHODE DE VISÉE À REVOIR APRÈS
     * TEST!*********************************************************
     * 
     * @param targetPosition  La position de la cible en Ta (limelight)
     * @param maxSpeed        La vitesse maximale du mouvement
     * @param maxAcceleration L'accélération maximale du mouvement
     * @param closedLoopSlot  le slot de contrôle à utiliser pour le PID
     *                        (généralement kSlot0)
     * @return la commande qui exécute ce mouvement
     */
    public Command goToTarget(double targetPosition, double maxSpeed, double maxAcceleration,
            ClosedLoopSlot closedLoopSlot) {
        if (!initDone) {
            return new InstantCommand(() -> System.out.println("Not initialized. Ignoring command"));
        }
        turretConstraints = new TrapezoidProfile.Constraints(maxSpeed, maxAcceleration);

        var command = new TrapezoidProfileMovement(turretMotor, targetPosition, turretConstraints, () -> 0.0,
                closedLoopSlot);
        command.addRequirements(this);
        CommandScheduler.getInstance().schedule(command);
        return command;
    }

    public void requestTurretAngle(double angle) {
        // if request is in normal range, go to request
        // if request is out of range but close to extremes, go to extremes
        // if request is out of range and too far from extremes, go to middle
        var target = angle;
        if (angle > TurretConstants.kMaxSetpoint) {
            if (angle <= (TurretConstants.kMaxSetpoint + TurretConstants.kExtremesThreshold)) {
                // over but close to max -> max
                target = TurretConstants.kMaxSetpoint;
            } else {
                // too far over -> middle
                target = 0;
            }
        } else if (angle < TurretConstants.kMinSetpoint) {
            if (angle >= (TurretConstants.kMinSetpoint - TurretConstants.kExtremesThreshold)) {
                // under but close to min -> min
                target = TurretConstants.kMinSetpoint;
            } else {
                // too far under -> middle
                target = 0;
            }
        }
        goToPosition(target);
    }

    private void goToPosition(double position) {
        turretController.setSetpoint(position, ControlType.kPosition);
    }

    public void safeStop() {
        setMotorSpeed(0);
    }
}
