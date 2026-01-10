package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedConstants;
import frc.robot.commands.SimpleLedPattern;
import frc.robot.commands.DisabledLedPattern;

public class Led extends SubsystemBase {
    private AddressableLED m_led;
    public AddressableLEDBuffer m_buffer;

    public Led(CoralTool coralTool) {
        // LEDPattern.solid(Color.kOrange).blink(Second.of(0.25));
        // LEDPattern.rainbow(100, 100).scrollAtRelativeSpeed(Hertz.of(.5));
        m_led = new AddressableLED(LedConstants.kPwmPort);
        m_buffer = new AddressableLEDBuffer(LedConstants.kNumLeds);
        m_led.setLength(LedConstants.kNumLeds);
        m_led.start();

        setDefaultCommand(new DisabledLedPattern(this, coralTool));
    }

    @Override
    public void periodic() {
        // Periodically send the latest LED color data to the LED strip for it to
        // display
        m_led.setData(m_buffer);
    }

    /**
     * Creates a command that runs a pattern on the entire LED strip.
     *
     * @param pattern the LED pattern to run
     *
     */
    public Command runPattern(LEDPattern pattern) {
        return new SimpleLedPattern(this, pattern);
    }
}
