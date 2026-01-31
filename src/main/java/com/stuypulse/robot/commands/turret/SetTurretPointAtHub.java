package com.stuypulse.robot.commands.turret;

import com.stuypulse.robot.subsystems.turret.Turret;
import com.stuypulse.robot.subsystems.turret.Turret.TurretState;

public class SetTurretPointAtHub extends SetTurretState {
    public SetTurretPointAtHub() {
        
        super(TurretState.POINT_AT_HUB);
       
        addRequirements(Turret.getInstance());
    }
}
