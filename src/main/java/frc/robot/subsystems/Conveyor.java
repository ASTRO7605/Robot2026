package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Conveyor extends SubsystemBase {
    // initialisation du moteur et de l'encodeur
    private SparkMax ConvMotor  = new SparkMax(ConveyorConstants.conveyorMotorId, MotorType.kBrushless);
    private RelativeEncoder convEncoder = ConvMotor.getEncoder();
    private SparkMaxConfig currentConfig;


    public Conveyor() {
        // checkInit();
        // configutation du moteur (le temp d'attente de réponse du moteur)
        ConvMotor.setCANTimeout(Constants.kCANTimeout);
        ConvMotor.setPeriodicFrameTimeout(Constants.kPeriodicFrameTimeout);

        currentConfig = new SparkMaxConfig();
        currentConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        currentConfig.inverted(false);
        // limitation du courant et de la tension pour protéger le moteur et la batterie
        currentConfig.voltageCompensation(ConveyorConstants.kVoltageCompensation);
        currentConfig.smartCurrentLimit(ConveyorConstants.kCurrentLimit);

        
        ConvMotor.configure(currentConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // fait tourner les roues du convoyeur pour faire entrer les balles
    public void converyorWheelsIn(){
        ConvMotor.set(ConveyorConstants.kInSpeed);
    }

    // fait tourner les roues du convoyeur pour faire sortir les balles
    public void conveyorWheelsOut(){
        ConvMotor.set(ConveyorConstants.kOutSpeed);
    }

    // arrête les roues du convoyeur
    public void ConveyorWheelOff(){
        ConvMotor.stopMotor();
    }

    public boolean isConveyorStopped() {
        return Math.abs(convEncoder.getVelocity()) < ConveyorConstants.kThresholdMotorStopped;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ConveyorVelocity", convEncoder.getVelocity());
        SmartDashboard.putBoolean("ConveyorStopped", isConveyorStopped());
    }

}
