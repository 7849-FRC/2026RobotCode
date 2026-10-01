package frc.robot.cmds;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import lib.Commands.TimedCommand;

public class ShootAtSetSpeedTimed extends TimedCommand {

    private final Shooter shooter;
    private final Indexer indexer;
    private final double rps;

    public ShootAtSetSpeedTimed(Shooter shooter, Indexer indexer, double rps, double seconds,
            Subsystem... requirements) {
        super(seconds, requirements);

        this.shooter = shooter;
        this.indexer = indexer;
        this.rps = rps;
    }

    @Override
    public void init() {

    }

    @Override
    public void exec() {
        shooter.setVelocity(rps).run();

        if (shooter.getRPS() >= rps) {
            indexer.stage1On().run();
            indexer.runFeeder(40).run();
        }
    }

    @Override
    public void end() {
        shooter.stop();
        indexer.stage1Off();
    }

}
