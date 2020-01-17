package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;


public class HPMC {
    static final float NANOSECONDS_PER_SECOND = 1000000000;
    static final double LOOK_AHEAD_TIME = 0.075;
    static final double FINE_POWER_SCALE =  0.0002;
    static final double MAX_POWER_CHANGE = 2.0;
    static final long MS_PER_NS = 1000000;

    int historySize = 3;
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
    long smAccelerationTicks = 0;
    long smDesiredMoved = 0;
    long smStartStopping = 0;
    long smDesiredPosition = 0;
    long smDecelerationTicks = 0;
    long smDecelerationTick = 0;



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

    public void setPower(double powerIn) {
        power = powerIn;
        motor.setPower(power);
    }

    public void setSpeed(double newDesiredSpeed) {
        desiredSpeed = newDesiredSpeed;
        autoAdjust();
    }

    public void updateCurrentSpeed() {
        position = motor.getCurrentPosition();
        long nanotime = System.nanoTime();
        //Don't update if it's less than 10 ms since the last update
        if ( (nanotime - lastUpdateTime) <   (10 * MS_PER_NS) ) {
            debug("Skipping double update");
            return;
        } else if (nanotime - lastUpdateTime > (100* MS_PER_NS) ) {
            debug("position and time lists are outdated.  Clearing: " + (nanotime-lastUpdateTime)/1000000.0);
            positionList.clear();
            timeList.clear();
        }
        positionList.add(position);
        timeList.add(nanotime);
        lastUpdateTime = nanotime;
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
                debug("Not enough history.  Using zeros");
                currentSpeed = 0;
                accelleration = 0;
            }
        } else {
            currentSpeed = 0;
            accelleration=0;
        }
    }


    public void autoAdjust() {
        autoAdjust(false);
    }

    public void manualAdjust(double speed) {
        power = speed / maxSpeed;
        if (power > 1) power = 1;
        if (power < -1) power = -1;
        motor.setPower(power);
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

        /*
        if (Math.abs(change) > MAX_POWER_CHANGE) {
            if (change < 0) {
                change = -MAX_POWER_CHANGE;
            } else {
                change = MAX_POWER_CHANGE;
            }
        }*/

        power = power + change;
        //debug(String.format("Power: %.4f change: %.4f speedSoon: %.2f  desiredSpeed: %.2f diff: %.2f pos: %d", power, change, speedSoon, desiredSpeed, difference,position));
        if (power > 1) power = 1;
        if (power < -1) power = -1;
        motor.setPower(power);

    }

    public void smoothMoveSetup(double distance, double power, double accelerationTicks, double decelerationTicks, Direction direction, boolean endStopped) {
        updateCurrentSpeed();
        smStartPosition = position;
        smDistance = (long) Math.abs(distance);
        if (direction == Direction.FORWARD) {
            long endPosition = smStartPosition + smDistance;
        } else {
            long endPosition = smStartPosition - smDistance;
        }
        desiredSpeed = currentSpeed;
        smStartSpeed = currentSpeed;
        smAccelerationTicks = (long) accelerationTicks;
        smAccelerationTick = 0;
        smDesiredMoved = 0;
        smDesiredPosition = position;
        smDecelerationTicks = (long) decelerationTicks;
        smDecelerationTick = 0;

        if (smAccelerationTicks<1) { smAccelerationTicks = 1;}
        if (direction == Direction.FORWARD) {
            smSpeed = power*maxSpeed;
        } else {
            smSpeed = -power*maxSpeed;
        }
        smState = MoveState.ACCELERATING;
        double  distanceDecelerating = (smSpeed / 2 ) * ((accelerationTicks+1) * tickTime / 1000.0);
        smStartStopping = (long) (smDistance - Math.abs( distanceDecelerating) );
        debug(String.format("Setting Up SM:  Distance: %d Power: %.2f  AT: %d D: %s SD %.2f   Start Slowing At:  %d", smDistance, power, smAccelerationTicks, direction.toString(), distanceDecelerating, smStartStopping));
        if (smStartStopping < (smDistance *0.4) ) {
            smStartStopping = (long) (smDistance *0.4);
        }
    }


    public boolean smTick() {
        double tickSeconds = tickTime / 1000.0;
        updateCurrentSpeed();
        long moved = Math.abs(position - smStartPosition);
        if ((moved > (smDistance-3) ) && (smState != MoveState.DONE)) {
            debug(String.format("DONE  Moved: %d of %d", moved, smDistance));
            stop();
            smState = MoveState.DONE;
            return(false);
        }
        switch (smState) {
            case ACCELERATING:
                smAccelerationTick++;
                double percentSpeed = (smAccelerationTick) / (double) smAccelerationTicks;


                //Set the desiredSpeed to the start speed plus a fraction of the desired difference in speed.
                if (false) { //new method
                    percentSpeed = Math.sqrt(percentSpeed);  //hmm..mabye not right

                    desiredSpeed = (long) (percentSpeed * smSpeed);
                    smDesiredPosition += desiredSpeed * tickSeconds;
                    smDesiredMoved += Math.abs(desiredSpeed * tickSeconds);
                    //double nextDesiredPosition = smDesiredPosition + (desiredSpeed * tickSeconds);
                    //desiredSpeed = (smDesiredPosition - position) / tickSeconds;
                    //desiredSpeed = recoverLostSign(desiredSpeed, smSpeed);
                    long behind = moved - smDesiredMoved;
                    if (moved < smDesiredMoved - 10) {
                        debug("FASTER:"+ behind);

                        desiredSpeed = smSpeed * 1.1;
                    } else if (moved > smDesiredMoved + 10) {
                        debug("WoA THERE " + behind);
                        desiredSpeed = 0.9 * smSpeed;
                    } else {
                        desiredSpeed = smSpeed;
                    }
                    manualAdjust(desiredSpeed);
                    debug(String.format("Spd: (%.1f of %.1f) Moved: (%d of %d) Behind: %d", currentSpeed, desiredSpeed, moved, smDesiredMoved, smDesiredMoved - moved));
                } else { //old better method
                    if (smAccelerationTick == 1) {
                        power = smSpeed / maxSpeed * 0.2;
                    }
                    double estimatedSpeed = (smAccelerationTick-1) / (double) smAccelerationTicks;
                    //This is what we want to move, but we target faster because acceleration takes time.
                    smDesiredMoved += Math.abs(estimatedSpeed* smSpeed * tickSeconds);

                    percentSpeed = (smAccelerationTick + 1) / (double) smAccelerationTicks;
                    percentSpeed = (percentSpeed * 0.8) + 0.2;  //start at 20% speed and ramp up from there
                    desiredSpeed = ((smSpeed - smStartSpeed) * (percentSpeed)) + smStartSpeed;

                    autoAdjust(true);

                    debug(String.format("Spd: (%.1f of %.1f) Moved: %d  DesiredMoved:%d Behind: %d", currentSpeed, desiredSpeed, moved, smDesiredMoved, smDesiredMoved - moved));

                }
                if (moved > smStartStopping) {
                    smState = MoveState.SLOWING;
                } else if (smAccelerationTick >= smAccelerationTicks) {
                    smState = MoveState.AT_SPEED;
                }
                if (true) {
                } else {

                }
                debug(String.format("ACC %%S: %.2f DS: %.2f  CS: %.2f Moved: %d(%d) of %d PWR:%.2f", percentSpeed, desiredSpeed, currentSpeed, moved, smDesiredMoved, smStartStopping, power));
                return (true);
            case AT_SPEED:
                smDesiredPosition += smSpeed * tickSeconds;
                smDesiredMoved += Math.abs(smSpeed * tickSeconds);
                if (true) { //old
                    //Increase speed by 10% if this wheel is behind;
                    long behind = moved - smDesiredMoved;
                    if (moved < smDesiredMoved - 10) {
                        debug("FASTER " + behind);
                        desiredSpeed = smSpeed * 1.1;
                    } else if (moved > smDesiredMoved + 10) {
                        debug("WoA THERE " + behind);
                        desiredSpeed = 0.9 * smSpeed;
                    } else {
                        desiredSpeed = smSpeed;
                    }
                    autoAdjust(true);
                } else {
                    smDesiredPosition += smSpeed * tickSeconds;
                    smDesiredMoved += Math.abs(smSpeed * tickSeconds);
                    desiredSpeed = (smDesiredPosition - position) / tickSeconds;
                    manualAdjust(desiredSpeed);
                }
                if (moved > smStartStopping ) {
                    smState = MoveState.SLOWING;
                }
                debug(String.format("A_S DS:%.2f  CS:%.2f Moved:%d of %d PWR:%.2f", desiredSpeed, currentSpeed, moved, smStartStopping, power));
                return(true);
            case SLOWING:
                double distanceDecelerating = smDistance - smStartStopping;
                long stoppingDistanceLeft =  (smDistance - moved);
                smDecelerationTick++;
                long ticksLeft = smDecelerationTicks - smDecelerationTick;
                if  (ticksLeft < 1) {
                    stop();
                    smState = MoveState.DONE;
                    return (false);
                }
                //double ratio = (0.7 * smDecelerationTick  / (double) smDecelerationTicks) + 0.1;
                //desiredSpeed = (smSpeed  - (smSpeed * ratio));

                double percentLeft = ((stoppingDistanceLeft) / (double) distanceDecelerating);
                double timeLeft = (ticksLeft * tickSeconds);
                desiredSpeed =  (stoppingDistanceLeft / timeLeft * 2.2) + 50;
                desiredSpeed = recoverLostSign(desiredSpeed, smSpeed);

                //estamated stopping speed is 500/50ms;
                long minStoppingDistance = (long) (( currentSpeed / 300 ) * tickSeconds);

                if ( stoppingDistanceLeft  > minStoppingDistance ) {

                    autoAdjust(true);
                    debug(String.format("SLO DSPD: %.2f  SPD: %.2f DistLeft: %d TL:%d M:%d of %d PWR:%.2f", desiredSpeed, currentSpeed, stoppingDistanceLeft, ticksLeft, moved, smDistance, power));
                    return(true);
                } else {
                    stop();
                    debug(String.format("BRK DSPD: %.2f  SPD: %.2f DistLeft: %d TL:%d M:%d of %d PWR:%.2f", desiredSpeed, currentSpeed, stoppingDistanceLeft, ticksLeft, moved, smDistance, power));

                    return (true);
                }
            case DONE:
                debug(String.format("DONE DSPD: %.2f  SPD: %.2f M:%d of %d PWR:%.2f", desiredSpeed, currentSpeed,  moved, smDistance, power));

                return(false);
        }
        return false;
    }

    double recoverLostSign(double number, double source) {
        if (source<0) {
            return(-number);
        }
        return number;
    }

    void stop() {
        power=0;
        motor.setPower(0);
        desiredSpeed = 0;
    }

    public void setTickTime(long tickTimeIn) { tickTime = tickTimeIn;}
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

    void debug(String string) {
        if (label != null) {
            System.out.println("M: " + label + " " + string);
        }
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
