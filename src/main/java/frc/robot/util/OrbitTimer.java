package frc.robot.util;

public class OrbitTimer {

    private long startTime;

    public OrbitTimer() {
        this.startTime = 0;
    }

    public void start() {
        this.startTime = System.currentTimeMillis();
    }

    public long getTimeDeltaMillis() {
        return System.currentTimeMillis() - this.startTime;
    }

    public double getTimeDeltaSec() {
        double deltaT = (System.currentTimeMillis() - this.startTime) / 1000.0;
        return deltaT;
    }
    
}
