package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;


public class HPMC {
    static final float NANOSECONDS_PER_SECOND = 1000000000;
    static final double LOOK_AHEAD_TIME = 0.075;
    static final double FINE_POWER_SCALE =  0.0001;
    static final double ROUGH_POWER_SCALE = 0.0001;
    static final long ENCODERS_PER_INCH = 100;
    static final double MAX_POWER_CHANGE = 0.2;
    static final int DEBUG = 1;

    //disable all automatic speed
    boolean simpleMode = false;


    //History size should probably be odd.
    int historySize = 5;

    static long tickTime = 50; //in milliseconds
    //For Smart Ticks
    private long nextWake = 0;




    private int position = 0;
    private double desiredSpeed = 0;
    double currentSpeed = 0;
    double power = 0;
    double accelleration = 0;
    float updatesPerSecond = 0;
    double speedSoon = 0;
    double change = 0;
    double maxSpeed = 0;
    long smAccelerationTick = 0;
    long lastUpdateTime = 0;
    DcMotor motor = null;

    //The name of the motor for debugging;
    String label = null;

    public enum MoveState { ACCELERATING, AT_SPEED, SLOWING, DONE};
    public enum Direction {FORWARD, REVERSE};

    //smooth move variables
    MoveState smState = MoveState.DONE;
    long smStartPosition = 0;
    double smStartSpeed = 0;
    double smSpeed = 0;
    long  smDistance = 0;
    double smAccelerationTicks = 0;
    long smDesiredMoved = 0;
    long smStartStopping = 0;


    ArrayList<Integer> positionList = new ArrayList<Integer>();
    ArrayList<Long> timeList = new ArrayList<Long>();

    public HPMC(DcMotor setMotor) {
        motor = setMotor;
    }

    public HPMC(HardwareMap hardwareMap, String motorString, double maxSpeedIn) {
        motor = hardwareMap.get(DcMotor.class, motorString);
        maxSpeed = maxSpeedIn;
    }

    public void setDirection(DcMotor.Direction direction) {

        motor.setDirection(direction);
        updateCurrentSpeed();
    }

    public HPMC(DcMotor setMotor, double maxSpeedIn) {
        motor = setMotor;
        maxSpeed = maxSpeedIn;
    }

    public void setMaxSpeed(double maxSpeed) {
        this.maxSpeed = maxSpeed;
    }

    public void setPower(double power) {
        if (simpleMode) {
            motor.setPower(power);
        } else {
            setSpeed(power*maxSpeed);
        }
    }

    public void setSpeed(double newDesiredSpeed) {
        desiredSpeed = newDesiredSpeed;
        autoAdjust();
    }

    public void updateCurrentSpeed() {
        position = motor.getCurrentPosition();
        long nanotime = System.nanoTime();
        //Don't update if it's less than 10 ms since the last update
        if ( (nanotime - lastUpdateTime) <  10000000) {
            System.out.println("Skipping double update");
            return;
        }

        positionList.add(position);
        timeList.add(nanotime);

        if (positionList.size() > historySize) {
            positionList.remove(0);
        }
        if (timeList.size() > historySize) {
            timeList.remove(0);
        }
        //set currentSpeed to
        if (timeList.size() > 2) {
            int start = 0;
            int end = timeList.size() - 1;
            int middle = end / 2;

            long elapsed = timeList.get(end) - timeList.get(start);
            int distance = positionList.get(end) - positionList.get(start);
            long oldDistance = positionList.get(middle) - positionList.get(start);
            long newDistance = positionList.get(end) - positionList.get(middle);
            long oldElapsed = timeList.get(middle) - timeList.get(start);
            long newElapsed = timeList.get(end) - timeList.get(middle);
            if ((elapsed > 0) && (oldElapsed > 0) && (newElapsed > 0)) {
                currentSpeed = distance * (NANOSECONDS_PER_SECOND / elapsed);
                updatesPerSecond = (timeList.size() * (NANOSECONDS_PER_SECOND / elapsed));
                double oldSpeed = oldDistance * (NANOSECONDS_PER_SECOND / oldElapsed);
                double newSpeed = newDistance * (NANOSECONDS_PER_SECOND / newElapsed);
                double secondsElapsed = (newElapsed / NANOSECONDS_PER_SECOND);

                accelleration = (newSpeed - oldSpeed) / secondsElapsed;
            } else {
                currentSpeed = 0;
                accelleration = 0;
            }
        } else {
            currentSpeed = 0;
        }
    }


    public void autoAdjust() {
        autoAdjust(false);
    }

    public void autoAdjust(boolean skipUpdateSpeed) {
        if (!skipUpdateSpeed) {
            updateCurrentSpeed();
        }
        speedSoon = currentSpeed + (accelleration * LOOK_AHEAD_TIME);
        double difference =  desiredSpeed - speedSoon;
        if ( Math.abs(desiredSpeed) < 1) {
            change = -power;
            //System.out.println(String.format("Change from zeroing: %.4f", change));
        } else if ( (speedSoon /  desiredSpeed) > 1.01) {
            change =  (power * (desiredSpeed / speedSoon)) - power;
            //System.out.println(String.format("Change from overspeed: %.4f", change));
        } else {
            change = FINE_POWER_SCALE * difference;
            //System.out.println(String.format("Change from power scale: %.4f", change));
        }

        if (Math.abs(change) > MAX_POWER_CHANGE) {
            if (change < 0) {
                change = -MAX_POWER_CHANGE;
            } else {
                change = MAX_POWER_CHANGE;
            }
        }

        power = power + change;
        if (label != null && DEBUG > 2) {
            System.out.println(String.format("M: %s Power: %.4f change: %.4f speedSoon: %.2f  desiredSpeed: %.2f diff: %.2f pos: %d", label, power, change, speedSoon, desiredSpeed, difference,position));
        }
        if (power > 1) power = 1;
        if (power < -1) power = -1;
        motor.setPower(power);

    }

    public void smoothMoveSetup(double distance, double power, double accelerationTicks, Direction direction, boolean endStopped) {
        updateCurrentSpeed();
        smStartPosition = position;
        smDistance = (long) Math.abs(distance * ENCODERS_PER_INCH);
        if (direction == Direction.FORWARD) {
            long endPosition = smStartPosition + smDistance;
        } else {
            long endPosition = smStartPosition - smDistance;
        }
        desiredSpeed = currentSpeed;
        smStartSpeed = currentSpeed;
        smAccelerationTicks = accelerationTicks;
        smAccelerationTick = 0;
        if (smAccelerationTicks<1) { smAccelerationTicks = 1;}
        if (direction == Direction.FORWARD) {
            smSpeed = power*maxSpeed;
        } else {
            smSpeed = -power*maxSpeed;
        }
        smState = MoveState.ACCELERATING;
        double  distanceDecelerating = (smSpeed / 2 ) * (accelerationTicks * tickTime / 1000);
        smStartStopping = (long) (smDistance - Math.abs( distanceDecelerating) );
        if (label != null) {
            System.out.println(String.format("Setting Up M: %s  Distance: %d Power: %.2f  AT: %d D: %s Stopping Distance %.2f   Start Slowing At:  %d", label, smDistance, power, accelerationTicks, direction.toString(), distanceDecelerating, smStartStopping));
        }
        if (smStartStopping > (smDistance / 2) ) {
            smStartStopping = (long) (smDistance / 2);
        }
    }


    public boolean smTick() {
        updateCurrentSpeed();
        long moved = Math.abs(position - smStartPosition);
        if (moved > smDistance ) {
            desiredSpeed = 0;
            autoAdjust(true);
            smState = MoveState.DONE;
            return(false);
        }
        switch (smState) {
            case ACCELERATING:
                smAccelerationTick++;
                //Set the desiredSpeed to the start speed plus a fraction of the desired difference in speed.
                double percentSpeed = (smAccelerationTick + 2 ) / smAccelerationTicks;
                percentSpeed = (percentSpeed * 0.7) + 0.3;  //start at 30% speed and ramp up from there
                desiredSpeed = ((smSpeed - smStartSpeed) * (percentSpeed)) + smStartSpeed;
                smDesiredMoved += Math.abs(desiredSpeed * (tickTime/1000));
                if (smAccelerationTick >= smAccelerationTicks) {
                    smState=MoveState.AT_SPEED;
                }
                if (label != null) {
                    System.out.println(String.format("M: %s ACC PS: %.2f DS: %.2f  CS: %.2f", label, percentSpeed, desiredSpeed, currentSpeed));
                }

                autoAdjust(true);
                return (true);
            case AT_SPEED:
                smDesiredMoved += Math.abs(desiredSpeed  * (tickTime/1000));
                //Increase speed by 10% if this wheel is behind;
                if ( moved < smDesiredMoved - 100) {
                    desiredSpeed = desiredSpeed*1.1;
                }
                autoAdjust(true);
                if (moved > smStartStopping ) {
                    smAccelerationTick = 0;
                    smState = MoveState.SLOWING;
                }
                if (label != null) {
                    System.out.println(String.format("M: %s A_S PS: %.2f DS: %.2f  CS: %.2f", label, desiredSpeed, currentSpeed));
                }
                return (true);
            case SLOWING:
                smAccelerationTick++;
                double ratio = smAccelerationTick / smAccelerationTicks;
                desiredSpeed = (smSpeed  - (smSpeed * ratio));
                if (smAccelerationTick >= smAccelerationTicks) {
                    smState = MoveState.DONE;
                    desiredSpeed = 0;
                }
                if (label != null) {
                    System.out.println(String.format("M: %s SLO PS: %.2f DS: %.2f  CS: %.2f", label, desiredSpeed, currentSpeed));
                }
                autoAdjust(true);
                return (true);
            case DONE:
                motor.setPower(0);
                return(false);
        }
        return false;
    }




    public void setLabel(String string) { label = string;}
    public void setHistorySize(int size) { historySize = size;}
    public double getPower() { return power; }
    public double getCurrentSpeed() { return currentSpeed; }
    public double getDesiredSpeed() { return desiredSpeed;}
    public double getAccelleration()  { return accelleration; }
    public double getSpeedSoon()  { return speedSoon; }
    public double getUpdatesPerSecond() { return updatesPerSecond;}
    public int getCurrentPosition() { return position; }
    public String getSMStatus() {
        return String.format("State: %s  Moved: %d Speed: %.2f  Desired: %.2f Power: %.2f StoppingAt: %d",
                smState.toString(),
                Math.abs(position - smStartPosition),
                currentSpeed, desiredSpeed, power, smStartStopping);
    }




    public void tickSleep() {
        long now = System.nanoTime();
        nextWake = nextWake + tickTime * 1000000;
        if (nextWake < now) {
            nextWake = now + tickTime * 1000000;
            double msLate = (now - nextWake) / 1000000;
            if (msLate > 100) {
                System.out.println(String.format("Either this is the first tick or something is wrong: %.1f ms late", msLate));
                throw new RuntimeException(String.format("Can't keep up: tick took %.1f ms too long", msLate));
            }
            return;
        }
        long sleepTime = (int) Math.floor((nextWake - now) / 1000000);
        if (false) {//for debugging
            System.out.println("Sleeping: " + sleepTime);
        }

        try {
            Thread.sleep(sleepTime);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
