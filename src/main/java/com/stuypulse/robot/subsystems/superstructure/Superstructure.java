      package com.stuypulse.robot.subsystems.superstructure;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Superstructure extends SubsystemBase {
    public static final Superstructure instance;

    static {
            instance = new SuperstructureImpl();
    }

    public static Superstructure getInstance() {
        return instance; 
    }

    public enum SuperstructureState {
        // add ShooterState constructor and potential states for the game
        STOP(Settings.Superstructure.SHOOTERINTAKE_STOP, Settings.Superstructure.SHOOTERINTAKE_STOP),
        SHOOT(Settings.Superstructure.SHOOTERINTAKE_SHOOT, Settings.Superstructure.SHOOTERINTAKE_SHOOT),
        INTAKE(Settings.Superstructure.SHOOTERINTAKE_INTAKE, Settings.Superstructure.SHOOTERINTAKE_INTAKE),
        OUTTAKE(Settings.Superstructure.SHOOTERINTAKE_OUTTAKE, Settings.Superstructure.SHOOTERINTAKE_OUTTAKE);

        private double shooterState;
        private double indexerState;

        private SuperstructureState(double shooterState, double indexerState){
            this.shooterState = shooterState;
            this.indexerState = indexerState;                            
        }

        public double getShooterState(){
            return shooterState;
        }
        
        public double getIndexerState(){
            return indexerState;
        }
    }

    protected SuperstructureState state;

    protected Superstructure(){
        this.state = SuperstructureState.STOP;
    }

    public void setState(SuperstructureState state ){
        this.state = state;
    }

    public SuperstructureState getState(){
        return state;
    }
}
