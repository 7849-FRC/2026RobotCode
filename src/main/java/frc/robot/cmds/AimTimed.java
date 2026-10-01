package frc.robot.cmds;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import lib.Commands.TimedCommand;

public class AimTimed extends TimedCommand {
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentricFacingAngle aiming;
    private final Vision vision;

    public AimTimed(CommandSwerveDrivetrain drivetrain, Vision vision, SwerveRequest.FieldCentricFacingAngle aiming,
            double seconds,
            Subsystem... requirments) {
        super(seconds, requirments);

        this.drivetrain = drivetrain;
        this.aiming = aiming;
        this.vision = vision;
    }

    @Override
    public void init() {

    }

    @Override
    public void exec() {
        if (!vision.hasValidTargets()) {
            return;
        }
        
        drivetrain.setControl(aiming
                .withTargetDirection(vision.calculateRobotOffsetToTargetCenter(drivetrain.getPose().getRotation())));
    }

    @Override
    public void end() {

    }

}
