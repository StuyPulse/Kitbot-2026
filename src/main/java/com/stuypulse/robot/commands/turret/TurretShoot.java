package com.stuypulse.robot.commands.turret;

import com.stuypulse.robot.subsystems.turret.Turret.TurretState;

public class TurretShoot extends TurretSetState {
    public TurretShoot() {
        super(TurretState.SHOOTING);
    }
}