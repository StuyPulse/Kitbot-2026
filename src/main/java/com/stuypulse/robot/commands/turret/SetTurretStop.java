package com.stuypulse.robot.commands.turret;

import com.stuypulse.robot.subsystems.turret.Turret.TurretState;

public class SetTurretStop extends SetTurretState {
    public SetTurretStop() {
        super(TurretState.STOP);
    }
}
