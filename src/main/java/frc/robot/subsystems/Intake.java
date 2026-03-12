package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TurretConstants;

public class Intake extends SubsystemBase {
    private SparkMax IntakeMotor = new SparkMax(IntakeConstants.rightIntakeMotorId, MotorType.kBrushless);
    private RelativeEncoder intakeEncoder = IntakeMotor.getEncoder();

    private SparkClosedLoopController intakeController = IntakeMotor.getClosedLoopController();
    private SparkLimitSwitch limitSwitch = IntakeMotor.getReverseLimitSwitch();
    private SparkMaxConfig currentConfig;

    private boolean initDone = false;
    private boolean lastSwitchState = false;

    private TrapezoidProfile.Constraints turretConstraints;

    //constructeur du sous-système
    public Intake(){

    }

    public double computeFF() {
        return (IntakeConstants.kMaxFF * Math.cos(Math.toRadians(getPosition())));
    }

    
    public double getPosition() {
        return intakeEncoder.getPosition();
    }
}
