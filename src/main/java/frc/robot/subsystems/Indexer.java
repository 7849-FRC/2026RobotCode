package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.NiceSubsytem;

public final class Indexer extends SubsystemBase implements NiceSubsytem {
    private static Indexer instance = null;

    public static Indexer getInstance() {
        if (instance == null) {
            instance = new Indexer();
        }

        return instance;
    }

    private final TalonFX stage1;
    private final TalonFX feeder;

    private final edu.wpi.first.wpilibj.Timer oscillateTimer = new edu.wpi.first.wpilibj.Timer();

    private Indexer() {

        stage1 = new TalonFX(Constants.Indexer.STAGE1_MOTOR_PORT);

        final TalonFXConfiguration stage1Con = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(InvertedValue.Clockwise_Positive))
                .withCurrentLimits(
                        new CurrentLimitsConfigs().withStatorCurrentLimit(50).withStatorCurrentLimitEnable(true));

        stage1.getConfigurator().apply(stage1Con);

        // // Cap output voltage to 8V (tune this down as needed)
        // VictorSPXConfiguration stage1Config = new VictorSPXConfiguration();
        // stage1Config.voltageCompSaturation = 8.0;
        // stage1.configAllSettings(stage1Config);
        // stage1.enableVoltageCompensation(true);

        feeder = new TalonFX(Constants.Indexer.FEEDER_MOTOR_PORT);

        final Slot0Configs feederPID = new Slot0Configs().withKP(Constants.Indexer.FEEDER_P)
                .withKA(Constants.Indexer.FEEDER_A).withKV(Constants.Indexer.FEEDER_V);

        final TalonFXConfiguration config = new TalonFXConfiguration().withSlot0(feederPID)
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(InvertedValue.Clockwise_Positive))
                .withCurrentLimits(
                        new CurrentLimitsConfigs().withStatorCurrentLimit(40).withStatorCurrentLimitEnable(true));

        feeder.getConfigurator().apply(config);
    }

    public Runnable stage1On() {
        return () -> stage1.set(0.5);
    }

    public Runnable stage1Back() {
        return () -> stage1.set(-0.5);
    }

    public Runnable stage1Off() {
        return () -> {
            oscillateTimer.stop();
            oscillateTimer.reset();
            stage1.set(0);
        };
    }

    public Runnable oscillateStage1() {
        return () -> {
            if (!oscillateTimer.isRunning()) {
                oscillateTimer.start();
            }

            int ms = (int) (oscillateTimer.get() * 1000);

            // Full cycle: 2000ms forward oscillating 75-100%, then 200ms reverse
            int phase = ms % 2200;

            if (phase >= 2000) {
                // Reverse kick for 200ms
                stage1.set(-0.75);
            } else {
                // Oscillate between 75% and 100% every 400ms
                double speed = (phase % 400 < 200) ? 1.0 : 0.75;
                stage1.set(speed);
            }
        };
    }

    public Runnable runFeeder(double rps) {
        final VelocityVoltage velocityVoltage = new VelocityVoltage(rps).withSlot(0);

        return () -> {
            feeder.setControl(velocityVoltage);
        };
    }

    public Runnable runFeederBack() {
        return () -> {
            feeder.set(-0.5);
        };
    }

    public Runnable stopFeeder() {
        return () -> {
            feeder.stopMotor();
        };
    }

    @Override
    public void initialize() {

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Feeder RPS", feeder.getVelocity().getValueAsDouble());
    }
}
