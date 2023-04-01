package frc.robot.autos;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoBalance {
    public BuiltInAccelerometer mRioAccel;
    public AHRS mNavx; 
    public int state;
    public int debounceCount;
    public int wobbleCount; 
    private double robotSpeedSlow;
    private double robotSpeedFast;
    private double onChargeStationDegree;
    private double levelDegree;
    private double debounceTime;
    private double singleTapTime;
    private double scoringBackUpTime;
    private double doubleTapTime;

    public AutoBalance(DrivetrainSubsystem dt) {
        mRioAccel = new BuiltInAccelerometer();
        this.mNavx = dt.getNavx(); 
        state = 0;
        debounceCount = 0;
        wobbleCount = 0; 

        /**********
         * CONFIG *
         **********/
        // Speed the robot drived while scoring/approaching station, default = 0.4
        robotSpeedFast = 0.5;

        // Speed the robot drives while balancing itself on the charge station.
        // Should be roughly half the fast speed, to make the robot more accurate,
        // default = 0.2
        robotSpeedSlow = 0.22;

        // Angle where the robot knows it is on the charge station, default = 13.0
        onChargeStationDegree = 13.0;

        // Angle where the robot can assume it is level on the charging station
        // Used for exiting the drive forward sequence as well as for auto balancing,
        // default = 6.0
        levelDegree = 6.0;

        // Amount of time a sensor condition needs to be met before changing states in
        // seconds
        // Reduces the impact of sensor noice, but too high can make the auto run
        // slower, default = 0.2
        debounceTime = 0.19;

        // Amount of time to drive towards to scoring target when trying to bump the
        // game piece off
        // Time it takes to go from starting position to hit the scoring target
        singleTapTime = 0.4;

        // Amount of time to drive away from knocked over gamepiece before the second
        // tap
        scoringBackUpTime = 0.2;

        // Amount of time to drive forward to secure the scoring of the gamepiece
        doubleTapTime = 0.3;

    }

    public double getPitch() {
        return Math.atan2((-mRioAccel.getX()),
                Math.sqrt(mRioAccel.getY() * mRioAccel.getY() + mRioAccel.getZ() * mRioAccel.getZ())) * 57.3;
        // return mNavx.getPitch(); 
    }

    public double getRoll() {
        return Math.atan2(mRioAccel.getY(), mRioAccel.getZ()) * 57.3;
        //return mNavx.getRoll(); 
    }

    // returns the magnititude of the robot's tilt calculated by the root of
    // pitch^2 + roll^2, used to compensate for diagonally mounted rio
    public double getTilt() {
        double pitch = getPitch();
        double roll = getRoll();
        if ((pitch + roll) >= 0) {
            return Math.sqrt(pitch * pitch + roll * roll);
        } else {
            return -Math.sqrt(pitch * pitch + roll * roll);
        }
    }

    public int secondsToTicks(double time) {
        return (int) (time * 50);
    }

    // routine for automatically driving onto and engaging the charge station.
    // returns a value from -1.0 to 1.0, which left and right motors should be set
    // to.
    public double autoBalanceRoutine() {
        //System.out.println("Before Switch");
        switch (state) {
            // drive forwards to approach station, exit when tilt is detected
            case 0:
                //System.out.println("In case staement 0");
                if (getTilt() < -onChargeStationDegree) {
                    debounceCount++;
                }
                if (debounceCount > secondsToTicks(debounceTime)) {
                    state = 1;
                    debounceCount = 0;
                    return robotSpeedSlow;
                }
                return robotSpeedFast;
            // driving up charge station, drive slower, stopping when level
            case 1:
                if (Math.abs(getTilt()) < levelDegree) {
                    debounceCount++;
                }
                if (debounceCount > secondsToTicks(debounceTime)) {
                    state = 2;
                    debounceCount = 0;
                    return 0;
                }
                return robotSpeedSlow;
            case 2: 
                // on charge station, stop motors and wait for end of auto 
                if (Math.abs(getTilt()) <= levelDegree / 2) {
                    debounceCount++;
                }
                if (debounceCount > secondsToTicks(debounceTime)) {
                    state = 3;
                    debounceCount = 0;
                    return 0;
                }

                // if not level yet...
                if (getTilt() <= -levelDegree) {
                    System.out.println("Positive Speed");
                    return robotSpeedSlow / 1.5;
                } else if (getTilt() >= levelDegree) {
                    System.out.println("Negative Speed");
                    return -(robotSpeedSlow / 1.5);
                }
            case 3:
                return 0.0;
        }
        return 0.0;
    }
}
