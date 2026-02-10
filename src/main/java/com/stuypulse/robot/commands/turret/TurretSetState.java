package com.stuypulse.robot.commands.turret;

import com.stuypulse.robot.subsystems.turret.Turret;
import com.stuypulse.robot.subsystems.turret.Turret.TurretState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class TurretSetState extends InstantCommand {
     
    private final Turret turret;
    private final TurretState state;

    public TurretSetState(TurretState state) {
        this.turret = Turret.getInstance();
        this.state = state;

        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.setState(state);
    }
}