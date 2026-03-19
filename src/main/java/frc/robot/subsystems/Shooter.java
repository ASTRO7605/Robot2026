package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends SubsystemBase {
    private SparkMax rightShootMotor = new SparkMax(ShooterConstants.rightShooterMotorId, MotorType.kBrushless);
    private SparkMax leftShootMotor = new SparkMax(ShooterConstants.leftShooterMotorId, MotorType.kBrushless);

    private RelativeEncoder rightShootEncoder = rightShootMotor.getEncoder();
    private RelativeEncoder leftShootEncoder = leftShootMotor.getEncoder();

    private SparkClosedLoopController rightShootController = rightShootMotor.getClosedLoopController();
    private SparkClosedLoopController leftShootController = leftShootMotor.getClosedLoopController();
    private double kp = ShooterConstants.kp;
    private double kd = ShooterConstants.kd;
    private double ki = ShooterConstants.ki;
    private double kv = ShooterConstants.kv;

    // table de calcul de la vitesse en fonction de la distance
    InterpolatingDoubleTreeMap distanceTable = new InterpolatingDoubleTreeMap();

    // pour le PID
    private SparkMaxConfig currentConfig;

    public Shooter() {
        // configuration du moteur (le temp d'attente de réponse du moteur)
        rightShootMotor.setCANTimeout(Constants.kCANTimeout);
        rightShootMotor.setPeriodicFrameTimeout(Constants.kPeriodicFrameTimeout);

        FeedForwardConfig feedForwardConfig = new FeedForwardConfig();
        feedForwardConfig.kV(ShooterConstants.kv);

        currentConfig = new SparkMaxConfig();
        var leftConfig = new SparkMaxConfig();

        currentConfig.idleMode(IdleMode.kBrake);
        currentConfig.inverted(false);
        // configuration du pid2q
        currentConfig.closedLoop
                .p(ShooterConstants.kp)
                .i(ShooterConstants.ki)
                .d(ShooterConstants.kd);
        currentConfig.closedLoop.apply(feedForwardConfig);
        // limitation du courant et de la tension pour protéger le moteur et la batterie
        currentConfig.voltageCompensation(Constants.kVoltageCompensation);
        currentConfig.smartCurrentLimit(ShooterConstants.kCurrentLimit);

        // Conversion des unités de l'encodeur
        currentConfig.encoder.positionConversionFactor(ShooterConstants.fPositionConversion);
        currentConfig.encoder.velocityConversionFactor(ShooterConstants.fVelocityConversion);

        rightShootMotor.configure(currentConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftConfig.follow(rightShootMotor, true);
        leftShootMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Remplissage de la table de distance et de vitesse
        distanceTable.put(0.0, 0.0);
        distanceTable.put(1.0, 1000.0);
        distanceTable.put(2.0, 2000.0);
        distanceTable.put(3.0, 3000.0); // Remplacez ces valeurs par les distances et vitesses réelles
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(getSubsystem() + ".RightShootVelocity", rightShootEncoder.getVelocity());
        SmartDashboard.putNumber(getSubsystem() + ".LeftShootVelocity", leftShootEncoder.getVelocity());
        SmartDashboard.putNumber("shooterRightAppliedOutput", rightShootMotor.getAppliedOutput());
        SmartDashboard.putNumber("shooterLeftAppliedOutput", leftShootMotor.getAppliedOutput());
        SmartDashboard.putNumber("shooterRightCurrent", rightShootMotor.getOutputCurrent());
        SmartDashboard.putNumber("shooterLeftCurrent", leftShootMotor.getOutputCurrent());
        SmartDashboard.putNumber(getSubsystem() + ".encoderPosition", getPosition());
        kp = SmartDashboard.getNumber("shooter.kP", ShooterConstants.kp);
        ki = SmartDashboard.getNumber("shooter.kI", ShooterConstants.ki);
        kd = SmartDashboard.getNumber("shooter.kD", ShooterConstants.kd);
        kv = SmartDashboard.getNumber("shooter.kV", ShooterConstants.kv);

            SparkMaxConfig newConfig = new SparkMaxConfig();
    newConfig.closedLoop
        .p(kp)
        .i(ki)
        .d(kd);

    FeedForwardConfig ff = new FeedForwardConfig();
    ff.kV(kv);
    newConfig.closedLoop.apply(ff);

    // Apply config to the motor
    rightShootMotor.configure(newConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftConfig.follow(rightShootMotor, true);
    leftShootMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    // fait rouler les moteurs à partir du controleur
    public void setManualMotorSpeed(double speed) {

        rightShootController.setSetpoint(
                speed,
                ControlType.kVelocity,
                ClosedLoopSlot.kSlot0);
    }

    public void stopMotors(){
        rightShootMotor.stopMotor();
    }

    // controle le moteur directement avec le voltage
    public void setMotorVoltage(double voltage) {
        leftShootMotor.setVoltage(voltage);
        rightShootMotor.setVoltage(voltage);
    }

    /**
     * Retourne la position actuelle de l'encodeur
     * 
     * @return
     */
    public double getPosition() {
        return rightShootEncoder.getPosition();
    }

    public double GetShooterInterpolatingSpeed(double distance) {
        return distanceTable.get(distance);
    }

    /**
     * Fait rouler les moteurs à la vitesse calculée à partir de la distance
     * 
     * @param distance la distance à laquelle le robot doit tirer
     */
    public void shootByDistance(double distance) {
        double speed = GetShooterInterpolatingSpeed(distance);
        setManualMotorSpeed(speed);
    }

    public void freezeAllMotorFunctions() {
        rightShootMotor.stopMotor();
        leftShootMotor.stopMotor();
    }

    public void safeStop() {
        setManualMotorSpeed(0);
    }
}
