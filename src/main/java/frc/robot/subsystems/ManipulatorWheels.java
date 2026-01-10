package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ManipulatorWheelsConstants;
import frc.robot.Constants.PDHConstants;
import frc.robot.utils.PDH;

public class ManipulatorWheels extends SubsystemBase {
    private SparkMax motor = new SparkMax(ManipulatorWheelsConstants.kMotorId, MotorType.kBrushless);
    private RelativeEncoder encoder = motor.getEncoder();
    // private SparkClosedLoopController controller =
    // motor.getClosedLoopController();

    public ManipulatorWheels() {
        motor.setCANTimeout(Constants.kCANTimeout);
        motor.setPeriodicFrameTimeout(Constants.kPeriodicFrameTimeout);

        var config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast);
        config.inverted(true);
        config.voltageCompensation(ManipulatorWheelsConstants.kVoltageCompensation);
        config.smartCurrentLimit(ManipulatorWheelsConstants.kCurrentLimit);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SmartDashboard.putNumber("manipulator wheel PDH current", 0);
    }

    public void turnOnWheelsIn() {
        motor.set(ManipulatorWheelsConstants.kInPower);
    }

    public void turnOnWheelsOut() {
        motor.set(ManipulatorWheelsConstants.kOutPower);
    }

    public void turnOffWheels() {
        motor.stopMotor();
    }

    public boolean isWheelsStopped() {
        return Math.abs(encoder.getVelocity()) < ManipulatorWheelsConstants.kThresholdMotorStopped;
    }

    public void freezeAllMotorFunctions() {
        motor.stopMotor();
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("wheelsVelocity", encoder.getVelocity());
        // SmartDashboard.putNumber("manipulatorWheelsCurrent",
        // PDH.getInstance().getChannelCurrent(PDHConstants.kManipulatorWheelsChannel));
    }
}
