package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LedConstants;
import frc.robot.subsystems.CoralTool;
import frc.robot.subsystems.Led;

public class DisabledLedPattern extends Command {
    private AddressableLEDBufferView m_alliView;
    private AddressableLEDBufferView m_coralView;
    private CoralTool m_coralTool;
    private LEDPattern m_redAlliPattern;
    private LEDPattern m_blueAlliPattern;
    private LEDPattern m_noAlliPattern;
    private LEDPattern m_noInitPattern;
    private LEDPattern m_initDonePattern;

    public DisabledLedPattern(Led led, CoralTool coralTool) {
        addRequirements(led);
        m_alliView = led.m_buffer.createView(0, (LedConstants.kNumLeds / 2) - 1);
        m_coralView = led.m_buffer.createView((LedConstants.kNumLeds / 2), LedConstants.kNumLeds - 1);
        m_coralTool = coralTool;
        m_redAlliPattern = LEDPattern.solid(Color.kRed)
                .breathe(LedConstants.kBreatheCycle);
        m_blueAlliPattern = LEDPattern.solid(Color.kBlue)
                .breathe(LedConstants.kBreatheCycle);
        m_noAlliPattern = LEDPattern.solid(Color.kBlue).blink(LedConstants.kNoAllianceBlink)
                .overlayOn(LEDPattern.solid(Color.kRed));
        m_noInitPattern = LEDPattern.solid(Color.kYellow).breathe(LedConstants.kBreatheCycle);
        m_initDonePattern = LEDPattern.solid(Color.kGreen);

    }

    @Override
    public void execute() {
        final var alli = DriverStation.getAlliance();
        LEDPattern alliPattern = (alli.isEmpty()) ? m_noAlliPattern : (alli.get() == Alliance.Red) ? m_redAlliPattern :

                m_blueAlliPattern;

        LEDPattern coralPattern = m_coralTool.isInitDone() ? m_initDonePattern : m_noInitPattern;
        alliPattern.applyTo(m_alliView);
        coralPattern.applyTo(m_coralView);
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
