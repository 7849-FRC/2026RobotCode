package frc.robot.cmds;

import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class BounceIntakeDown extends Command {
    private final Intake intake;

    public BounceIntakeDown(Intake intake) {
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        if (!intake.getState().equals(intake.getOutType())) {
            return;
        }

        
        intake.setPivotPositionControl(new PositionVoltage(Constants.Intake.intakeOutRotations));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}

