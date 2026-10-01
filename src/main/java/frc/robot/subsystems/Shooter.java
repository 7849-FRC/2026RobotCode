package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.NiceSubsytem;

public final class Shooter extends SubsystemBase implements NiceSubsytem {
    private static Shooter instance;

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }

        return instance;
    }

    // private final TalonFX left;
    private final TalonFX right;

    private Shooter() {
        // left = new TalonFX(Constants.Shooter.LEFT_KRAKEN_CANID);
        right = new TalonFX(Constants.Shooter.RIGHT_KRAKEN_CANID);

        final TalonFXConfiguration config = new TalonFXConfiguration()
                .withMotorOutput(
                        new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
                .withCurrentLimits(
                        new CurrentLimitsConfigs().withStatorCurrentLimit(100).withStatorCurrentLimitEnable(true));

        final Slot0Configs shooterConfigs = new Slot0Configs()
                .withKP(Constants.Shooter.P)
                .withKI(Constants.Shooter.I)
                .withKD(Constants.Shooter.D)
                .withKS(Constants.Shooter.S)
                .withKV(Constants.Shooter.V)
                .withKA(Constants.Shooter.A);

        right.clearStickyFaults();

        right.getConfigurator().apply(config.withMotorOutput(
                new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Coast)));

        right.getConfigurator().apply(shooterConfigs);
    }

    public Runnable setVelocity(double rps) {
        return () -> {
            final VelocityVoltage request = new VelocityVoltage(rps)
                    .withSlot(0);

            right.setControl(request);
        };
    }

    public Runnable stop() {
        return () -> {
            right.stopMotor();
        };
    }

    public Runnable runFullSpeedRaw() {
        return () -> {
            right.set(1);
        };
    }

    public double getRPS() {
        return right.getVelocity().getValueAsDouble();
    }

    @Override
    public void initialize() {

    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Left Velocity: ", left.getVelocity().getValueAsDouble());
        // SmartDashboard.putNumber("Right Velocity: ", right.getVelocity().getValueAsDouble());

        // SmartDashboard.putNumber("Shooter Power: ",
        //         (left.getVelocity().getValueAsDouble() / Constants.Shooter.SHOOTER_MAX_RPS) * 100);
    }
}
