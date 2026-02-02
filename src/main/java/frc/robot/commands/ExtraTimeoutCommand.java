package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * @author rhhs
 */
public abstract class ExtraTimeoutCommand extends Command {
    protected double m_extraOneTimeout = -1;
    protected double m_extraTwoTimeout = -1;
    private double m_startExtraOneTime;
    private double m_startExtraTwoTime;

    public ExtraTimeoutCommand() {
    }

    protected synchronized final void startExtraOneTimeout(double seconds) {
        if (seconds < 0) {
            throw new IllegalArgumentException("Seconds must be positive.  Given:" + seconds);
        }
        m_startExtraOneTime = Timer.getFPGATimestamp();
        m_extraOneTimeout = seconds;
    }

    protected synchronized final void startExtraTwoTimeout(double seconds) {
        if (seconds < 0) {
            throw new IllegalArgumentException("Seconds must be positive.  Given:" + seconds);
        }
        m_startExtraTwoTime = Timer.getFPGATimestamp();
        m_extraTwoTimeout = seconds;
    }

    public synchronized final double timeSinceExtraOneInitialized() {
        return m_startExtraOneTime < 0 ? 0 : Timer.getFPGATimestamp() - m_startExtraOneTime;
    }

    public synchronized final double timeSinceExtraTwoInitialized() {
        return m_startExtraTwoTime < 0 ? 0 : Timer.getFPGATimestamp() - m_startExtraTwoTime;
    }

    protected synchronized boolean isExtraOneTimedOut() {
        return m_extraOneTimeout != -1 && timeSinceExtraOneInitialized() >= m_extraOneTimeout;
    }

    protected synchronized boolean isExtraTwoTimedOut() {
        return m_extraTwoTimeout != -1 && timeSinceExtraTwoInitialized() >= m_extraTwoTimeout;
    }

    protected void resetExtraOneTimer() {
        m_extraOneTimeout = -1;
    }

    protected void resetExtraTwoTimer() {
        m_extraTwoTimeout = -1;
    }

}