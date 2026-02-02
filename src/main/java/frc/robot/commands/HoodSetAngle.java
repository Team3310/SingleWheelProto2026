package frc.robot.commands;

import frc.robot.subsystems.Shooter;


public class HoodSetAngle extends ExtraTimeoutCommand {
    private final Shooter shooter;
    private double angle;

    public HoodSetAngle(Shooter shooter, double angle) {
        this.shooter = shooter;
        this.angle = angle;
    }

    @Override
    public void initialize() {
        shooter.setHoodMotionMagicPositionAbsolute(angle);
        resetExtraOneTimer();
        startExtraOneTimeout(0.1);
    }

    @Override
    public boolean isFinished() {
        if (isExtraOneTimedOut() && shooter.hasFinishedHoodTrajectory()) {
            return true;
        }
        return false;
    }
}
