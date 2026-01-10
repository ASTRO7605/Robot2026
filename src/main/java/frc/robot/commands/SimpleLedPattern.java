package frc.robot.commands;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Led;

public class SimpleLedPattern extends Command {
    private Led m_led;
    private LEDPattern m_pattern;

    public SimpleLedPattern(Led led, LEDPattern pattern) {
        addRequirements(led);
        m_led = led;
        m_pattern = pattern;
    }

    @Override
    public void execute() {
        m_pattern.applyTo(m_led.m_buffer);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
