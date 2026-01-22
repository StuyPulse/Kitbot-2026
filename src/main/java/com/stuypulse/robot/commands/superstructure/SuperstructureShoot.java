package com.stuypulse.robot.commands.superstructure;

import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.superstructure.Superstructure.SuperstructureState;

public class SuperstructureShoot extends SuperstructureSetState {
    public SuperstructureShoot() {
        super(SuperstructureState.SHOOTING);
    }
}
