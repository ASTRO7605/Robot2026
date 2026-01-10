package frc.robot.utils;

import edu.wpi.first.wpilibj.PowerDistribution;

public class PDH {
    private static PDH instance;
    private PowerDistribution m_PDH;
    
    public PDH() {
        m_PDH = new PowerDistribution();
    }

    public double getChannelCurrent(int channel) {
        return m_PDH.getCurrent(channel);
    }

    public static PDH getInstance() {
        if (instance == null) {
            instance = new PDH();
        }
        return instance;
    }
}
