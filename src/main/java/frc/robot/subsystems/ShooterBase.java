package frc.robot.subsystems;
 
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterBaseConstants;
 
public class ShooterBase extends SubsystemBase {
    // initialisation du moteur et de l'encodeur
    private SparkMax ShooterBaseMotor  = new SparkMax(ShooterBaseConstants.shooterBaseMotorId, MotorType.kBrushless);
    private RelativeEncoder ShooterBaseEncoder = ShooterBaseMotor.getEncoder();
    private SparkMaxConfig currentConfig;
   
    public ShooterBase() {
     currentConfig = new SparkMaxConfig();
     currentConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
     currentConfig.inverted(true);
 
     // limitation du courant et de la tension pour protéger le moteur et la batterie
        currentConfig.voltageCompensation(Constants.kVoltageCompensation);
        currentConfig.smartCurrentLimit(ShooterBaseConstants.kCurrentLimit);
       
        ShooterBaseMotor.configure(currentConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
 
    // fait tourner les roues de la shooter base pour faire entrer et apporter les balles
    public void ShooterBaseWheelsIn(){
        System.out.println("Button pressed: ");
        ShooterBaseMotor.set(ShooterBaseConstants.kInSpeed);
    }

    public void ShooterBaseWheelsOut(){
        System.out.println("Button pressed: ");
        ShooterBaseMotor.set(ShooterBaseConstants.kOutSpeed);
    }
 
     // arrête les roues du shooter
    public void ShooterBaseWheelOff(){
        ShooterBaseMotor.stopMotor();
    }
 
    public boolean isShooterBaseStopped() {
        return Math.abs(ShooterBaseEncoder.getVelocity()) < ShooterBaseConstants.kThresholdMotorStopped;
    }
 
    @Override
    public void periodic() {
       
    }
 
}