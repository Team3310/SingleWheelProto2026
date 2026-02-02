package frc.robot.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    /* ================= ENUMS ================= */

    public enum HoodControlMode {
        MOTION_MAGIC,
        MANUAL,
        MOTION_MAGIC_TRACK_LIMELIGHT
    }

    /* ================= CONSTANTS ================= */

    // Gear ratios
    private final double HOOD_OUTPUT_TO_ENCODER_RATIO = 322.0 / 20.0;

    private final double HOOD_DEGREES_PER_ROTATION = 360.0 / HOOD_OUTPUT_TO_ENCODER_RATIO;

    /* ================= MOTORS ================= */

    private final TalonFX shooterMainMaster = new TalonFX(Constants.SHOOTER_MAIN_MOTOR_MASTER_CAN_ID);
    // private final TalonFX shooterMainSlave = new TalonFX(Constants.SHOOTER_MAIN_MOTOR_SLAVE_CAN_ID);
    private final TalonFX shooterKicker = new TalonFX(Constants.SHOOTER_KICKER_MOTOR_CAN_ID);
    private final TalonFX shooterIntake = new TalonFX(Constants.SHOOTER_INTAKE_MOTOR_CAN_ID);
    private final TalonFX shooterHood = new TalonFX(Constants.SHOOTER_HOOD_MOTOR_CAN_ID);

    /* ================= CONTROL REQUESTS ================= */

    private final DutyCycleOut percentOut = new DutyCycleOut(0);
    private final VelocityVoltage velocityOut = new VelocityVoltage(0);
    private final MotionMagicVoltage hoodMM = new MotionMagicVoltage(0).withFeedForward(0.07);

    /* ================= STATE ================= */

    private HoodControlMode hoodControlMode = HoodControlMode.MANUAL;
    private double homePositionAngleDegrees = Constants.HOOD_COMPETITION_HOME_POSITION_DEGREES;
    private double targetHoodRotations = 0;
    private boolean isReady;

    private float mainKp = 0.5f;
    private float mainKi = 0.0f;
    private float mainKd = 0.0f;
    private float mainKv = 0.05f;
    private float mainKa = 0.0f;
    private float mainKs = 0.0f;

    private float mainTargetRPM = 0f;
    private float kickerTargetRPM = 0f;
    private float intakeTargetRPM = 0f;

    /* ================= SINGLETON ================= */

    private static final Shooter INSTANCE = new Shooter();

    public static Shooter getInstance() {
        return INSTANCE;
    }

    /* ================= CONSTRUCTOR ================= */

    private Shooter() {
        configureMotors();
        SmartDashboard.putNumber("Kp", mainKp);
        SmartDashboard.putNumber("Ki", mainKi);
        SmartDashboard.putNumber("Kd", mainKd);
        SmartDashboard.putNumber("Kv", mainKv);
        SmartDashboard.putNumber("Ka", mainKa);
        SmartDashboard.putNumber("Ks", mainKs);
        SmartDashboard.putNumber("Main Targ RPM", mainTargetRPM);
        SmartDashboard.putNumber("Kicker Targ RPM", kickerTargetRPM);
        SmartDashboard.putNumber("Intake Targ RPM", intakeTargetRPM);
    }

    /* ================= CONFIGURATION ================= */

    private void configureMotors() {

        MotorOutputConfigs coastCCW = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.CounterClockwise_Positive);

        MotorOutputConfigs coastCW = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.Clockwise_Positive);

        MotorOutputConfigs brakeCW = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive);

        shooterMainMaster.getConfigurator().apply(coastCW);
        // shooterMainSlave.getConfigurator().apply(coastCCW);
        shooterKicker.getConfigurator().apply(coastCW);
        shooterIntake.getConfigurator().apply(coastCCW);
        shooterHood.getConfigurator().apply(brakeCW);

        // shooterMainSlave.setControl(
        //         new Follower(
        //                 shooterMainMaster.getDeviceID(),
        //                 MotorAlignmentValue.Opposed));

        CurrentLimitsConfigs shooterLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(100)
                .withStatorCurrentLimitEnable(false);

        CurrentLimitsConfigs hoodLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(30)
                .withStatorCurrentLimitEnable(true);

        shooterMainMaster.getConfigurator().apply(shooterLimits);
        // shooterMainSlave.getConfigurator().apply(shooterLimits);
        shooterKicker.getConfigurator().apply(shooterLimits);
        shooterIntake.getConfigurator().apply(shooterLimits);
        shooterHood.getConfigurator().apply(hoodLimits);

        Slot0Configs shooterPID = new Slot0Configs()
                .withKP(mainKp)
                .withKI(mainKi)
                .withKD(mainKd)
                .withKV(mainKv)
                .withKA(mainKa)
                .withKS(mainKs);

        shooterMainMaster.getConfigurator().apply(shooterPID);
        shooterKicker.getConfigurator().apply(
                shooterPID.withKP(0.5).withKV(0.145));
        shooterIntake.getConfigurator().apply(
                shooterPID.withKP(0.1));

        MotionMagicConfigs hoodMMCfg = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(50) // rps
                .withMotionMagicAcceleration(100); // rps²

        Slot0Configs hoodPID = new Slot0Configs()
                .withKP(0.9)
                .withKI(0.008)
                .withKD(0.0)
                .withKV(0.045);

        shooterHood.getConfigurator().apply(hoodMMCfg);
        shooterHood.getConfigurator().apply(hoodPID);
    }

    /* ================= SHOOTER ================= */

    public void setMainSpeed(double speed) {
        shooterMainMaster.setControl(percentOut.withOutput(speed));
    }

    public void setMainRPM(double rpm) {
        shooterMainMaster.setControl(
                velocityOut.withVelocity(rpm / 60.0));
    }

    public double getMainRPM() {
        return shooterMainMaster.getVelocity().getValueAsDouble() * 60.0;
    }

    public double getMainRotations() {
        return shooterMainMaster.getPosition().getValueAsDouble();
    }

    public void resetShooterPosition() {
        shooterMainMaster.setPosition(0);
    }

    /* ================= KICKER ================= */

    public void setKickerSpeed(double speed) {
        shooterKicker.setControl(percentOut.withOutput(speed));
    }

    public void setKickerRPM(double rpm) {
        shooterKicker.setControl(
                velocityOut.withVelocity(rpm / 60.0));
    }

    public double getKickerRPM() {
        return shooterKicker.getVelocity().getValueAsDouble() * 60.0;
    }

    /* ================= INTAKE ================= */

    public void setIntakeSpeed(double speed) {
        shooterIntake.setControl(percentOut.withOutput(speed));
    }

    public void setIntakeRPM(double rpm) {
        shooterIntake.setControl(
                velocityOut.withVelocity(rpm / 60.0));
    }

    public double getIntakeRPM() {
        return shooterIntake.getVelocity().getValueAsDouble() * 60.0;
    }

    /* ================= HOOD ================= */

    public void setHoodSpeed(double speed) {
        hoodControlMode = HoodControlMode.MANUAL;
        shooterHood.setControl(percentOut.withOutput(speed));
    }

    public void resetHoodHomePosition() {
        shooterHood.setPosition(0);
    }

    public double getHoodAngleAbsoluteDegrees() {
        return shooterHood.getPosition().getValueAsDouble()
                * HOOD_DEGREES_PER_ROTATION
                + homePositionAngleDegrees;
    }

    public void setHoodMotionMagicPositionAbsolute(double angle) {
        hoodControlMode = HoodControlMode.MOTION_MAGIC;

        double limitedAngle = limitHoodAngle(angle);
        double rotations = (limitedAngle - homePositionAngleDegrees)
                / HOOD_DEGREES_PER_ROTATION;

        targetHoodRotations = rotations;
        shooterHood.setControl(hoodMM.withPosition(rotations));
    }

    public boolean hasFinishedHoodTrajectory() {
        return Math.abs(
                shooterHood.getPosition().getValueAsDouble()
                        - targetHoodRotations) < 0.01;
    }

    private double limitHoodAngle(double targetAngle) {
        return Math.max(Constants.HOOD_MIN_ANGLE_DEGREES,
                Math.min(Constants.HOOD_MAX_ANGLE_DEGREES, targetAngle));
    }

    /* ================= STATE ================= */

    public void setReady(boolean ready) {
        isReady = ready;
    }

    public boolean isReady() {
        return isReady;
    }

    public void updateMainPid() {
        mainKp = (float)SmartDashboard.getNumber("Kp", 0.0);
        mainKi = (float)SmartDashboard.getNumber("Ki", 0.0);
        mainKd = (float)SmartDashboard.getNumber("Kd", 0.0);
        mainKv = (float)SmartDashboard.getNumber("Kv", 0.0);
        mainKa = (float)SmartDashboard.getNumber("Ka", 0.0);
        mainKs = (float)SmartDashboard.getNumber("Ks", 0.0);
        Slot0Configs shooterPID = new Slot0Configs()
                .withKP(mainKp)
                .withKI(mainKi)
                .withKD(mainKd)
                .withKV(mainKv)
                .withKA(mainKa)
                .withKS(mainKs);
        shooterMainMaster.getConfigurator().apply(shooterPID);
        shooterKicker.getConfigurator().apply(shooterPID);
    }

    public void updateRPM() {
        mainTargetRPM = (float)SmartDashboard.getNumber("Main Targ RPM", 0.0);
        kickerTargetRPM = (float)SmartDashboard.getNumber("Kicker Targ RPM", 0.0);
        intakeTargetRPM = (float)SmartDashboard.getNumber("Intake Targ RPM", 0.0);
        setMainRPM(mainTargetRPM);
        setKickerRPM(kickerTargetRPM);
        setIntakeRPM(intakeTargetRPM);
    }

    /* ================= PERIODIC ================= */

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Main RPM", getMainRPM());
        SmartDashboard.putNumber("Main RPM Graph", getMainRPM());
        SmartDashboard.putNumber("Kicker RPM", getKickerRPM());
        SmartDashboard.putNumber("Kicker RPM Graph", getKickerRPM());
        SmartDashboard.putNumber("Intake RPM", getIntakeRPM());
        SmartDashboard.putNumber("Intake RPM Graph", getIntakeRPM());
    }
}
