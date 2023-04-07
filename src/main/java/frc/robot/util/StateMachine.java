package frc.robot.util;

public class StateMachine {

    boolean atHome;
    boolean atMid;
    boolean atHigh;
    boolean atLow;
    boolean atIntake;

    public StateMachine() {
        atHome = false;
    }

    public void setAtHome(boolean atHome) {
        this.atHome = atHome;
    }

    public boolean getAtHome() {
        return this.atHome;
    }
    
}
