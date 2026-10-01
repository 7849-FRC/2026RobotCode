package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.NiceSubsytem;

public final class Intake extends SubsystemBase implements NiceSubsytem {
    private static Intake instance;

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }

        return instance;
    }

    public IntakeState currentState;
    public BounceState bounceState;
    public boolean shouldBeOut = false;

    private final TalonFX leftPivotMotor;
    private final TalonFX rightPivotMotor;
    private final WPI_VictorSPX intakeMotor;

    private Intake() {
        this.leftPivotMotor = new TalonFX(Constants.Intake.LEFT_PIVOT_MOTOR_PORT);
        this.rightPivotMotor = new TalonFX(Constants.Intake.RIGHT_PIVOT_MOTOR_PORT);
        this.intakeMotor = new WPI_VictorSPX(Constants.Intake.INTAKE_MOTOR_PORT);

        this.intakeMotor.setInverted(true);

        final VictorSPXConfiguration intakeConfig = new VictorSPXConfiguration();
        intakeConfig.voltageCompSaturation = 8.0;
        intakeMotor.configAllSettings(intakeConfig);
        intakeMotor.enableVoltageCompensation(true);

        this.currentState = IntakeState.IN;
        this.bounceState = BounceState.NOT;

        final TalonFXConfiguration configs = new TalonFXConfiguration()
                .withCurrentLimits(
                        new CurrentLimitsConfigs().withStatorCurrentLimit(40).withStatorCurrentLimitEnable(true))
                .withMotorOutput(
                        new MotorOutputConfigs()
                                .withNeutralMode(NeutralModeValue.Brake))
                .withSlot0(
                        new Slot0Configs()
                                .withKP(Constants.Intake.P)
                                .withKI(Constants.Intake.I)
                                .withKD(Constants.Intake.D)
                                .withKS(Constants.Intake.S)
                                .withKV(Constants.Intake.V));

        leftPivotMotor.getConfigurator().apply(
                configs.withMotorOutput(
                        new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive)));
        rightPivotMotor.getConfigurator().apply(configs);
    };

    public Runnable intake() {
        return () -> intakeMotor.set(1);
    }

    public Runnable outake() {
        return () -> intakeMotor.set(-1);
    }

    public Runnable stopIntake() {
        return () -> intakeMotor.set(0);
    }

    public Runnable zeroPivot() {
        return () -> leftPivotMotor.setPosition(0);
    }

    public Runnable setPivotPositionControl(PositionVoltage request) {
        return () -> {
            leftPivotMotor.setControl(request);

            rightPivotMotor
                    .setControl(new Follower(Constants.Intake.LEFT_PIVOT_MOTOR_PORT, MotorAlignmentValue.Opposed));

        };
    }

    public Runnable setBounceOff() {
        return () -> bounceState = BounceState.NOT;
    }

    public Runnable runPivotRawOut() {
        return () -> {
            leftPivotMotor.set(0.09);
            rightPivotMotor
                    .setControl(new Follower(Constants.Intake.LEFT_PIVOT_MOTOR_PORT, MotorAlignmentValue.Opposed));

            currentState = IntakeState.OUT;
        };
    }

    public Runnable runPivotRawIn() {
        return () -> {
            leftPivotMotor.set(-0.09);
            rightPivotMotor
                    .setControl(new Follower(Constants.Intake.LEFT_PIVOT_MOTOR_PORT, MotorAlignmentValue.Opposed));

            currentState = IntakeState.IN;
        };
    }

    public Runnable stopPivot() {
        return () -> {
            leftPivotMotor.stopMotor();
            rightPivotMotor.stopMotor();
        };
    }

    public void setCurrentState(IntakeState state) {
        currentState = state;
    }

    public IntakeState getState() {
        return currentState;
    }

    public double getPivotPosition() {
        return leftPivotMotor.getPosition().getValueAsDouble();
    }

    public Runnable runIntakeMotor() {
        return () -> {
            leftPivotMotor.set(-1);
        };
    }
    

    public Runnable runNewFeeder() {
        return () -> {
            rightPivotMotor.set(-0.3);
        };
    }

    public Runnable feedNewFeeder() {
        return () -> {
            rightPivotMotor.set(0.3);
        };
    }

    public Runnable stopIntakeMotor() {
        return () -> {
            leftPivotMotor.set(0);
        };
    }

    public Runnable stopNewFeeder() {
        return () -> {
            rightPivotMotor.set(0);
        };
    }

    @Override
    public void initialize() {

    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Intake State", currentState.toString());

        SmartDashboard.putNumber("Intake Position", leftPivotMotor.getPosition().getValueAsDouble());

        if (getPivotPosition() >= 17) {
            setCurrentState(Intake.IntakeState.OUT);
        } else if (getPivotPosition() <= 0) {
            setCurrentState(Intake.IntakeState.IN);
            zeroPivot();
        }

        if (!shouldBeOut && getPivotPosition() >= 3) {
            setPivotPositionControl(new PositionVoltage(0).withSlot(0)).run();
        }

        SmartDashboard.putBoolean("Should be out?", shouldBeOut);
    }

    public IntakeState getInType() {
        return IntakeState.IN;
    }

    public IntakeState getOutType() {
        return IntakeState.OUT;
    }

    public enum IntakeState {
        OUT,
        IN
    }

    public enum BounceState {
        NOT,
        UP,
        DOWN
    }
}
