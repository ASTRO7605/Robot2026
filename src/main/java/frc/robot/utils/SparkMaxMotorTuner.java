package frc.robot.utils;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkMaxConfigAccessor;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SparkMaxMotorTuner {
    private final String name;
    private final SparkMaxTunableMotorSystem subsSystem;

    public SparkMaxMotorTuner(SparkMaxTunableMotorSystem subsystem, String name) {
        this.name = name;
        this.subsSystem = subsystem;

        configToSmartDashboard(subsystem.getMotorConfig());

        if (subsystem.getProfileConstraints() != null) {
            profileConstraintsToSmartDashboard(subsystem.getProfileConstraints());
        }
    }

    public void configToSmartDashboard(SparkMaxConfigAccessor config) {
        SmartDashboard.putNumber(name + ".conf.P", config.closedLoop.getP());
        SmartDashboard.putNumber(name + ".conf.I", config.closedLoop.getI());
        SmartDashboard.putNumber(name + ".conf.D", config.closedLoop.getD());
    }

    public void profileConstraintsToSmartDashboard(TrapezoidProfile.Constraints constraints) {
        SmartDashboard.putNumber(name + ".conf.maxVelocity", constraints.maxVelocity);
        SmartDashboard.putNumber(name + ".conf.maxAcceleration", constraints.maxAcceleration);
    }

    public SparkMaxConfig configFromSmartDashboard() {
        final var p = SmartDashboard.getNumber(name + ".conf.P", 0);
        final var i = SmartDashboard.getNumber(name + ".conf.I", 0);
        final var d = SmartDashboard.getNumber(name + ".conf.D", 0);

        var config = new SparkMaxConfig();
        config.closedLoop.pid(p, i, d);

        return config;
    }

    public TrapezoidProfile.Constraints profileConstraintsFromSmartDashboard() {
        final var maxVelocity = SmartDashboard.getNumber(name + ".conf.maxVelocity", 0);
        final var maxAccel = SmartDashboard.getNumber(name + ".conf.maxAcceleration", 0);
        return new TrapezoidProfile.Constraints(maxVelocity, maxAccel);
    }

    public void apply() {
        subsSystem.apply(configFromSmartDashboard(), profileConstraintsFromSmartDashboard());
    }
}
