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
    private SparkMax ConvMotor  = new SparkMax(ConveyorConstants.conveyorMotorId, MotorType.kBrushless);

    //moteur pour le leftIntake (celui du bas)
    private SparkMax leftIntakeMotor = new SparkMax(ConveyorConstants.leftIntakeMotorId, MotorType.kBrushless);
    private RelativeEncoder convEncoder = ConvMotor.getEncoder();
    private RelativeEncoder intakeWheelEncoder = leftIntakeMotor.getEncoder();
    private SparkMaxConfig conveyorConfig;
    private SparkClosedLoopController intakeController = leftIntakeMotor.getClosedLoopController();
    private SparkClosedLoopController convController = ConvMotor.getClosedLoopController();
    // constructeur du sous-système
    public Conveyor() {
        // configutation du moteur (le temp d'attente de réonse du moteur)
        ConvMotor.setCANTimeout(Constants.kCANTimeout);
        ConvMotor.setPeriodicFrameTimeout(Constants.kPeriodicFrameTimeout);
        var intakeConfig = new SparkMaxConfig();
        leftIntakeMotor.setCANTimeout(Constants.kCANTimeout);
        leftIntakeMotor.setPeriodicFrameTimeout(Constants.kPeriodicFrameTimeout);

        conveyorConfig = new SparkMaxConfig();
        conveyorConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        conveyorConfig.inverted(true);

        conveyorConfig.encoder.positionConversionFactor(ConveyorConstants.fPositionConversion);
        conveyorConfig.encoder.velocityConversionFactor(ConveyorConstants.fVelocityConversion);

        intakeConfig = new SparkMaxConfig();
        intakeConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        intakeConfig.inverted(false);
        
        conveyorConfig.closedLoop

                .p(ConveyorConstants.kp)
                .i(ConveyorConstants.ki)
                .d(ConveyorConstants.kd);

        intakeConfig.closedLoop
                .p(ConveyorConstants.kp)
                .i(ConveyorConstants.ki)
                .d(ConveyorConstants.kd);

                Preferences.initDouble("Conveyor.kP", kp);
                Preferences.initDouble("Conveyor.kV", kv);
                Preferences.initDouble("Conveyor.kI", ki);
                Preferences.initDouble("Conveyor.kD", kd);


        if (SmartDashboard.getNumber("Conveyor.kP", -1.0) == -1.0) {
            // SmartDashboard.putNumber("Conveyor.kP", kp);
        }
            if (SmartDashboard.getNumber("Conveyor.kI", -1.0) == -1.0) {
            // SmartDashboard.putNumber("Conveyor.kI", ki);
     }
            if (SmartDashboard.getNumber("Conveyor.kD", -1.0) == -1.0) {
            // SmartDashboard.putNumber("Conveyor.kD", kd);
        }
        if (SmartDashboard.getNumber("Conveyor.kV", -1.0) == -1.0) {
            // SmartDashboard.putNumber("Conveyor.kV", kv);
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(getSubsystem() + ".ConveyorVelocity", convEncoder.getVelocity());
        SmartDashboard.putNumber(getSubsystem() + ".IntakeVelocity", intakeWheelEncoder.getVelocity());

        oldKd = kd;
        oldKi = ki;
        oldKp = kp;
        oldKv = kv;
        kp = Preferences.getDouble("Conveyor.kP", ConveyorConstants.kp);
        ki = Preferences.getDouble("Conveyor.kI", ConveyorConstants.ki);
        kd = Preferences.getDouble("Conveyor.kD", ConveyorConstants.kd);
        kv = Preferences.getDouble("Conveyor.kV", ConveyorConstants.kv);

        if(oldKd != kd ||oldKi != ki || oldKp !=kp || oldKv != kv){
SparkMaxConfig newConfig = new SparkMaxConfig();
    newConfig.closedLoop
        .p(kp)
        .i(ki)
        .d(kd);
        SparkMaxConfig leftConfig = new SparkMaxConfig();

    FeedForwardConfig ff = new FeedForwardConfig();
    ff.kV(kv);
    newConfig.closedLoop.apply(ff);

        // limitation du courant et de la tension pour protéger le moteur et la batterie
        conveyorConfig.voltageCompensation(Constants.kVoltageCompensation);
        conveyorConfig.smartCurrentLimit(ConveyorConstants.kCurrentLimit);
        newConfig.voltageCompensation(Constants.kVoltageCompensation);
        newConfig.smartCurrentLimit(ConveyorConstants.kCurrentLimit);

        ConvMotor.configure(conveyorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftIntakeMotor.configure(newConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
        SmartDashboard.putNumber("ConveyorVelocity", convEncoder.getVelocity());
        SmartDashboard.putBoolean("ConveyorStopped", isConveyorStopped());
        SmartDashboard.putNumber("intakeWheelVelocity", intakeWheelEncoder.getVelocity());
        SmartDashboard.putBoolean("IntakeStopped", isIntakeWheelStopped());
        SmartDashboard.putNumber("IntakeOutput", leftIntakeMotor.getAppliedOutput());
    
}

    // fait tourner les roues du intake pour faire entrer les balles *********************************************A FIX
    public void setConveyorWheelsSpeed(double speed){
        intakeController.setSetpoint(
            speed,
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0);
        convController.setSetpoint(
           speed,
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0);
    
    }

    
    // fait tourner les roues du intake pour faire entrer les balles
    public void conveyorWheelsIn(){
        ConvMotor.set(ConveyorConstants.setConveyorInSpeed);
        leftIntakeMotor.set(-ConveyorConstants.kIntakeInSpeed);
    }


    // fait tourner les roues du convoyeur pour faire sortir les balles
    public void conveyorWheelsOut(){
        ConvMotor.set(-ConveyorConstants.kOutSpeed);
        leftIntakeMotor.set(ConveyorConstants.kIntakeInSpeed);
    }

    // arrête les roues du convoyeur
    public void ConveyorWheelOff(){
        ConvMotor.stopMotor();
        leftIntakeMotor.stopMotor();
    }

    public boolean isConveyorStopped() {
        return Math.abs(convEncoder.getVelocity()) < ConveyorConstants.kThresholdMotorStopped;
    }
    public boolean isIntakeWheelStopped() {
        return Math.abs(intakeWheelEncoder.getVelocity()) < ConveyorConstants.kThresholdMotorStopped;
    }

    
    }


