package frc.robot.utils;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkMaxConfigAccessor;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public interface SparkMaxTunableMotorSystem {

    public SparkMaxConfigAccessor getMotorConfig();

    public TrapezoidProfile.Constraints getProfileConstraints();

    public void apply(SparkMaxConfig config, TrapezoidProfile.Constraints constraints);
}
