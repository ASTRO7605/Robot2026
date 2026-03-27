package frc.robot.commands;

import java.util.function.Supplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class TrapezoidProfileMovement extends Command {

    private final RelativeEncoder encoder;
    private final SparkClosedLoopController controller;

    private TrapezoidProfile profile;
    private final TrapezoidProfile.Constraints constraints;
    private final Timer timer = new Timer();
    private double initialPosition;
    private double initialVelocity;
    private final double targetPosition;
    private final Supplier<Double> feedForward;
    private final ClosedLoopSlot closedLoopSlot;

    public TrapezoidProfileMovement(SparkMax motor, double targetPosition,
            TrapezoidProfile.Constraints constraints, Supplier<Double> feedForward, ClosedLoopSlot closedLoopSlot) {
        encoder = motor.getEncoder();
        controller = motor.getClosedLoopController();
        this.constraints = constraints;
        this.targetPosition = targetPosition;
        this.feedForward = feedForward;
        this.closedLoopSlot = closedLoopSlot;
    }

    @Override
    public void initialize() {
        timer.restart();
        initialPosition = encoder.getPosition();
        initialVelocity = encoder.getVelocity();
        profile = new TrapezoidProfile(constraints);
    }

    @Override
    public void execute() {
        var currentTime = timer.get();
        var currentSetpoint = profile.calculate(
                currentTime,
                new State(initialPosition, initialVelocity),
                new State(targetPosition, 0));

        setState(currentSetpoint);
    }

    private void setState(TrapezoidProfile.State current) {
        controller.setSetpoint(
                current.position,
                ControlType.kPosition,
                closedLoopSlot,
                feedForward.get());
    }

    @Override
    public boolean isFinished() {
        return profile.isFinished(timer.get());
    }

}