package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;


public class HPMCVelocityFailure {
    static final float NANOSECONDS_PER_SECOND = 1000000000;
    static final double LOOK_AHEAD_TIME = 0.075;
    static final double FINE_POWER_SCALE =  0.0002;
    static final double MAX_POWER_CHANGE = 2.0;
    static final long MS_PER_NS = 1000000;


    int historySize = 3;
    static long tickTime = 50; //in milliseconds
    //For Smart Ticks
    private long nextWake = 0;
    private int currentPosition = 0;
    private double desiredVelocity = 0;
    double currentVelocity = 0;
    double power = 0;
    double accelleration = 0;
    float updatesPerSecond = 0;
    double velocitySoon = 0;
    double change = 0;
    double maxSpeed = 0;

    long lastUpdateTime = 0;
    DcMotorEx motor = null;

    //The name of the motor for debugging;
    String label = null;



    public enum MoveState { ACCELERATING, AT_SPEED, STOPPING, DONE, BRAKING, STUPID};
    public enum Direction {FORWARD, REVERSE};

    //smooth move variables
    MoveState smState = MoveState.DONE;
    long smAccelerationTick;
    long smStartPosition;
    long smEndPosition;
    double smStartSpeed;
    double smVelocity;
    long  smDistance;
    long smAccelerationTicks;
    long smDesiredProgress;
    long smStartStopping;
    long smDesiredPosition;
    long smDecelerationTicks;
    long smDecelerationTick;
    boolean smEndStopped = true;
    boolean smBrakingConfigured = false;




    ArrayList<Integer> positionList = new ArrayList<Integer>();
    ArrayList<Long> timeList = new ArrayList<Long>();

    public HPMCVelocityFailure(DcMotorEx setMotor) {
        motor = setMotor;
    }

    public HPMCVelocityFailure(HardwareMap hardwareMap, String motorString, double maxSpeedIn) {
        motor = hardwareMap.get(DcMotorEx.class, motorString);
        maxSpeed = maxSpeedIn;
    }

    public void setDirection(DcMotor.Direction direction) {
        motor.setDirection(direction);
        updateCurrentVelocity();
    }

    public HPMCVelocityFailure(DcMotorEx setMotor, double maxSpeedIn) {
        motor = setMotor;
        maxSpeed = maxSpeedIn;
    }

    public void setMaxSpeed(double maxSpeed) {
        this.maxSpeed = maxSpeed;
    }


    public void setPowerSmart(double powerFactor) {
        if (powerFactor > 1) powerFactor = 1;
        if (powerFactor < -1) powerFactor = -1;
        desiredVelocity = maxSpeed * powerFactor;
        autoAdjust();
    }

    public void setPower(double powerIn) {
        power = powerIn;
        motor.setPower(power);
    }

    public void setDesiredVelocity(double newDesiredVelocity) {
        desiredVelocity = newDesiredVelocity;
        autoAdjust();
    }

    public void updateCurrentVelocity() {
        currentPosition = motor.getCurrentPosition();
        long nanotime = System.nanoTime();
        //Don't update if it's less than 10 ms since the last update
        if ( (nanotime - lastUpdateTime) <   (10 * MS_PER_NS) ) {
            debug("Skipping double update");
            return;
        } else if (nanotime - lastUpdateTime > (100* MS_PER_NS) ) {
            debug("currentPosition and time lists are outdated.  Clearing: " + (nanotime-lastUpdateTime)/1000000.0);
            positionList.clear();
            timeList.clear();
        }
        positionList.add(currentPosition);
        timeList.add(nanotime);
        lastUpdateTime = nanotime;
        if (positionList.size() > historySize) {
            positionList.remove(0);
        }
        if (timeList.size() > historySize) {
            timeList.remove(0);
        }
        //set currentVelocity to
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
                currentVelocity = distance * (NANOSECONDS_PER_SECOND / elapsed);
                updatesPerSecond = (timeList.size() * (NANOSECONDS_PER_SECOND / elapsed));
                double oldSpeed = oldDistance * (NANOSECONDS_PER_SECOND / oldElapsed);
                double newSpeed = newDistance * (NANOSECONDS_PER_SECOND / newElapsed);
                double secondsElapsed = (newElapsed / NANOSECONDS_PER_SECOND);

                accelleration = (newSpeed - oldSpeed) / secondsElapsed;
            } else {
                debug("Not enough history.  Using zeros");
                currentVelocity = 0;
                accelleration = 0;
            }
        } else {
            currentVelocity = 0;
            accelleration=0;
        }
    }


    public void autoAdjust() {
        autoAdjust(false);
    }

    public void manualAdjust(double velocity) {
        power = velocity / maxSpeed;
        if (power > 1) power = 1;
        if (power < -1) power = -1;
        motor.setPower(power);
    }

    public void autoAdjust(boolean skipUpdateSpeed) {
        if (!skipUpdateSpeed) {
            updateCurrentVelocity();
        }
        velocitySoon = currentVelocity + (accelleration * LOOK_AHEAD_TIME);
        double difference =  desiredVelocity - velocitySoon;
        if ( Math.abs(desiredVelocity) < 1) {
            change = -power;
            //System.out.println(String.format("Change from zeroing: %.4f", change));
        } else if ( (velocitySoon / desiredVelocity) > 1.01) {
            change =  (power * (desiredVelocity / velocitySoon)) - power;
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
        //debug(String.format("Power: %.4f change: %.4f velocitySoon: %.2f  desiredVelocity: %.2f diff: %.2f pos: %d", power, change, velocitySoon, desiredVelocity, difference,currentPosition));
        if (power > 1) power = 1;
        if (power < -1) power = -1;
        motor.setPower(power);

    }


    public void smoothMoveSetup(double distance, double power, double accelerationTicks, double decelerationTicks, Direction direction, boolean endStopped) {
        updateCurrentVelocity();
        smStartPosition = currentPosition;

        if (distance < 0) {
            //reverse direction
            switch (direction) {
                case FORWARD:
                    direction = Direction.REVERSE;
                    break;
                case REVERSE:
                    direction = Direction.FORWARD;
            }
        }

        smDistance = (long) Math.abs(distance);

        if (direction == Direction.FORWARD) {
            smEndPosition = smStartPosition + smDistance;
        } else {
            smEndPosition = smStartPosition - smDistance;
        }

        desiredVelocity = currentVelocity;
        smStartSpeed = currentVelocity;
        smAccelerationTicks = (long) accelerationTicks;
        smAccelerationTick = 0;
        smDesiredProgress = 0;
        smDesiredPosition = currentPosition;
        smDecelerationTicks = (long) decelerationTicks;
        smDecelerationTick = 0;
        smEndStopped = endStopped;

        if (smAccelerationTicks<1) { smAccelerationTicks = 1;}
        if (direction == Direction.FORWARD) {
            smVelocity = power*maxSpeed;
        } else {
            smVelocity = -power*maxSpeed;
        }

        smState = MoveState.ACCELERATING;
        double  distanceDecelerating = (smVelocity / 2 ) * ((accelerationTicks+1) * tickTime / 1000.0);
        if (endStopped) {
            smStartStopping = (long) (smDistance - Math.abs(distanceDecelerating));
            if (smStartStopping < (smDistance *0.4) ) {
                smStartStopping = (long) (smDistance *0.4);
            }
        } else {
            smStartStopping = smDistance;
        }
        debug2(String.format("Setting Up SM:  Distance: %d Power: %.2f  AT: %d D: %s SD %.2f   Start Slowing At:  %d", smDistance, power, smAccelerationTicks, direction.toString(), distanceDecelerating, smStartStopping));

    }


    public String getMoveState() {
        String percent;
        if (smDistance > 0) {
            percent = String.format(" %.1f%% %d",  (double) moved()*100 / (double) smDistance, smDistance - moved() );
            percent = String.format(" %.1f%% %.1f %d",  (double) moved()*100 / (double) smDistance, currentVelocity * 100 / desiredVelocity , distanceLeft() );

        } else {
            percent = "---";
        }
        if (smState == MoveState.ACCELERATING) {
            return  percent + "-A";
        } else if (smState == MoveState.AT_SPEED) {
            return  percent + "-R";
        } else if ( smState == MoveState.BRAKING) {
            return  percent + "-B";
        } else if ( smState == MoveState.STOPPING) {
            return  percent + "-S" + (smDecelerationTicks - smDecelerationTick);
        } else if ( smState == MoveState.DONE) {
            return  percent + "-D";
        } else {
            return percent + "-?";
        }
    }

    public boolean smTick() {
        return smTick(1.0);
    }

    public boolean smTick(double speedAdjustment) {
        double tickSeconds = tickTime / 1000.0;
        updateCurrentVelocity();
        if (smState == MoveState.STUPID) {
            if (!motor.isBusy()) {
                //done;
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor.setPower(0);
                desiredVelocity = 0;
                motor.setVelocity(desiredVelocity);
                smState = MoveState.DONE;
                return(false);
            } else {
                return(true);
            }
        }
        if ((moved() > (smDistance-3) ) && (smState != MoveState.DONE)) {
            debug2(String.format("DONE  Moved: %d of %d", moved(), smDistance));
            stopIfEndingStopped();
            smState = MoveState.DONE;
            return(false);
        }
        //debug(String.format("Tick starting.  Moved %d of %d.  State: %s", moved(), smDistance, smState.toString() ));
        switch (smState) {
            case ACCELERATING:
                smAccelerationTick++;
                double targetPercentSpeed = (smAccelerationTick) / (double) smAccelerationTicks;
                double currentPercentVelocity = currentVelocity / smVelocity;


                if (smAccelerationTick == 1) {
                    power = smVelocity / maxSpeed * 0.2;
                } else if ((moved() > (smDistance / 3.0)) || (currentVelocity / smVelocity) > 0.9) {
                    debug(String.format("Switching to AT_SPEED  (%d > %d / 3) or ( %.1f / %.1f) > 0.9", moved(), smDistance, currentVelocity, smVelocity));
                    smState = MoveState.AT_SPEED;
                }
                if (currentPercentVelocity < -0.1) {
                    debug(String.format("RRRRT.  Going the wrong way: %.1f", currentPercentVelocity));
                    if ( Math.abs( (currentVelocity) / maxSpeed) < 0.1) {  //if greater than 10% speed, use brakes.
                        desiredVelocity = smVelocity;
                        autoAdjust(true);
                    }

                } else {
                    double estimatedPercentSpeed = (smAccelerationTick - 1) / (double) smAccelerationTicks;
                    //This is what we want to move, but we target faster because acceleration takes time.

                    smDesiredProgress += movedPerTick(estimatedPercentSpeed * smVelocity);
                    targetPercentSpeed = (smAccelerationTick + 1) / (double) smAccelerationTicks;
                    targetPercentSpeed = (targetPercentSpeed * 0.8) + 0.2;  //start at 20% speed and ramp up from there
                    desiredVelocity = ((smVelocity - smStartSpeed) * (targetPercentSpeed)) + smStartSpeed;
                    autoAdjust(true);
                }

                debug(String.format("ACC Spd: %.1f%% (%.1f of %.1f) Moved: %d  DesiredMoved:%d Behind: %d  Pow:%.1f", currentPercentVelocity * 100.0, currentVelocity, desiredVelocity, moved(), smDesiredProgress, smDesiredProgress - moved(), power));


                if (moved() > smStartStopping) {
                    if (smEndStopped) {
                        smState = MoveState.STOPPING;
                    } else {
                        smState = MoveState.DONE;
                        return(false);
                    }
                } else if (smAccelerationTick >= smAccelerationTicks) {
                    smState = MoveState.AT_SPEED;
                }

                //debug(String.format("ACC %.2f%% DS: %.2f  CS: %.2f Moved: %d(%d) of %d PWR:%.2f", targetPercentSpeed, desiredVelocity, currentVelocity, moved, smDesiredProgress, smStartStopping, power));
                return (true);
            case AT_SPEED:
                smDesiredPosition += distancePerTick(smVelocity);
                smDesiredProgress += movedPerTick(smVelocity);
                if (true) { //old
                    //Increase speed by 10% if this wheel is behind;
                    long ahead = smDesiredProgress - moved();
                    if (ahead <  -10) {
                        debug("FASTER " + ahead);
                        desiredVelocity = smVelocity * 1.1;
                    } else if (ahead > 10) {
                        if (ahead > (Math.abs(smVelocity) * tickSeconds * 4)) {  //more than 4 ticks ahead and more than 10 encoder counts ahead
                            long distanceLeft = smDistance - moved();
                            long movementPerTick = movedPerTick(smVelocity);
                            long ticksLeft = (smDistance - smDesiredProgress) / movementPerTick;
                            if (ticksLeft > 1) {
                                debug(String.format("FIXME: %d %d %d %d", distanceLeft, movementPerTick, smDistance, smDesiredProgress));
                                desiredVelocity = recoverLostSign(distanceLeft / ticksLeft, smVelocity);
                                debug(String.format("SLOW DOWN!  Left %d  TicksLeft %d  Desired Velocity %.1f", distanceLeft, ticksLeft, desiredVelocity));
                            } else {
                                debug("I should be done.  Why am I in this code");
                                desiredVelocity=smVelocity;
                            }
                        } else {
                            debug("WoA THERE " + ahead);
                            desiredVelocity = 0.9 * smVelocity;
                        }
                    } else {
                        desiredVelocity = smVelocity;
                    }
                    autoAdjust(true);
                } else {
                    smDesiredPosition += smVelocity * tickSeconds;
                    smDesiredProgress += Math.abs(smVelocity * tickSeconds);
                    desiredVelocity = (smDesiredPosition - currentPosition) / tickSeconds;
                    manualAdjust(desiredVelocity);
                }
                if (moved() > smStartStopping ) {
                    if (smEndStopped) {
                        smState = MoveState.STOPPING;
                    } else {
                        smState = MoveState.DONE;
                        return(false);
                    }
                }
                debug(String.format("A_S DS:%.2f  CS:%.2f Moved:%d of %d PWR:%.2f", desiredVelocity, currentVelocity, moved(), smStartStopping, power));
                return(true);
            case STOPPING:
                if (false) {
                    //Run_to_position mode.  I don't trust run to position.  It behaves pathalogically but it brakes beautifully.
                    smDecelerationTick++;
                    long ticksLeft = smDecelerationTicks - smDecelerationTick;
                    if (ticksLeft < 1) {
                        debug("TICKS LEFT IS LESS THAN ONE!");
                        stopIfEndingStopped();
                        smState = MoveState.DONE;
                        return (false);
                    }
                    if (!smBrakingConfigured) {
                        motor.setTargetPosition((int) smEndPosition);
                        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        motor.setPower(1);
                        smBrakingConfigured = true;
                    } else {
                        long stoppingDistanceLeft = (smDistance - moved());
                        double timeLeft = (ticksLeft * tickSeconds);
                        desiredVelocity = (stoppingDistanceLeft / timeLeft * 2.2) ;
                        if ( ( Math.abs(currentVelocity) < (maxSpeed / 5) )  || ( (desiredVelocity / currentVelocity) > 1.5 ) ) {
                            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            desiredVelocity = recoverLostSign(desiredVelocity, smVelocity);
                            autoAdjust();
                            debug(String.format("SLO DV: %.2f  AV: %.2f DistLeft: %d TL:%d M:%d of %d PWR:%.2f", desiredVelocity, currentVelocity, stoppingDistanceLeft, ticksLeft, moved(), smDistance, power));
                        } else {
                            debug(String.format("BRK DV: %.2f  AV: %.2f Ratio:%.2f", desiredVelocity, currentVelocity, desiredVelocity/currentVelocity));
                        }
                    }
                    return(true);

                } else if (false) {
                    long stoppingDistanceLeft = (smDistance - moved());
                    smDecelerationTick++;
                    long ticksLeft = smDecelerationTicks - smDecelerationTick;
                    if (ticksLeft < 1) {
                        stopIfEndingStopped();
                        smState = MoveState.DONE;
                        return (false);
                    }
                    //double ratio = (0.7 * smDecelerationTick  / (double) smDecelerationTicks) + 0.1;
                    //desiredVelocity = (smSpeed  - (smSpeed * ratio));

                    double timeLeft = (ticksLeft * tickSeconds);
                    desiredVelocity = (stoppingDistanceLeft / timeLeft * 3) ;
                    desiredVelocity = recoverLostSign(desiredVelocity, smVelocity);
                    //estiamated braking speed change is 600/50ms;
                    motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    //lie to the motor controller so it reacts faster
                    double energyRatio = currentVelocity * currentVelocity / (desiredVelocity * desiredVelocity) ;
                    double motorVelocity = desiredVelocity - energyRatio * 0.25 * (currentVelocity - desiredVelocity);
                    motor.setVelocity(motorVelocity);
                    debug(String.format("SLO DV: %.2f  AV: %.2f DistLeft: %d TL:%d M:%d of %d PWR:%.2f", desiredVelocity, currentVelocity, stoppingDistanceLeft, ticksLeft, moved(), smDistance, power));
                    return (true);
                } else {
                    long stoppingDistanceLeft = (smDistance - moved());
                    smDecelerationTick++;
                    long ticksLeft = smDecelerationTicks - smDecelerationTick - 1;
                    if (ticksLeft < 1) {
                        stopIfEndingStopped();
                        smState = MoveState.DONE;
                        return (false);
                    }
                    //double ratio = (0.7 * smDecelerationTick  / (double) smDecelerationTicks) + 0.1;
                    //desiredVelocity = (smSpeed  - (smSpeed * ratio));

                    double timeLeft = (ticksLeft * tickSeconds);
                    desiredVelocity = (stoppingDistanceLeft / timeLeft * 2) ;
                    desiredVelocity = recoverLostSign(desiredVelocity, smVelocity);

                    //estiamated braking speed change is 600/50ms;
                    long minStoppingDistance = (long) ((currentVelocity / 300) * tickSeconds);

                    if (stoppingDistanceLeft > minStoppingDistance) {

                        autoAdjust(true);
                        debug(String.format("SLO DSPD: %.2f  SPD: %.2f DistLeft: %d TL:%d M:%d of %d PWR:%.2f", desiredVelocity, currentVelocity, stoppingDistanceLeft, ticksLeft, moved(), smDistance, power));
                        return (true);
                    } else {
                        stopIfEndingStopped();
                        debug(String.format("BRK DSPD: %.2f  SPD: %.2f DistLeft: %d TL:%d M:%d of %d PWR:%.2f", desiredVelocity, currentVelocity, stoppingDistanceLeft, ticksLeft, moved(), smDistance, power));

                        return (true);
                    }
                }
            case DONE:
                //debug(String.format("DONE DSPD: %.2f  SPD: %.2f M:%d of %d PWR:%.2f", desiredVelocity, currentVelocity,  moved(), smDistance, power));
                stopIfEndingStopped();
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

    void stopIfEndingStopped() {
        if (smEndStopped) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            power = 0;
            motor.setPower(0);
            desiredVelocity = 0;
        }
    }

    public void setTickTime(long tickTimeIn) { tickTime = tickTimeIn;}
    public void setLabel(String string) { label = string;}
    public void setHistorySize(int size) { historySize = size;}
    public double getPower() { return power; }
    public double getCurrentVelocity() { return currentVelocity; }
    public double getDesiredVelocity() { return desiredVelocity;}
    public double getAccelleration()  { return accelleration; }
    public double getVelocitySoon()  { return velocitySoon; }
    public double getUpdatesPerSecond() { return updatesPerSecond;}
    public int getCurrentPosition() { return currentPosition; }
    public String getSMStatus() {
        return String.format("State: %s  Moved: %d Speed: %.2f  Desired: %.2f Power: %.2f StoppingAt: %d",
                smState.toString(),
                Math.abs(currentPosition - smStartPosition),
                currentVelocity, desiredVelocity, power, smStartStopping);
    }

    void debug(String string) {
        if (label != null) {
            System.out.println("M: " + label + " " + string);
        }
    }
    void debug2(String string) {
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

    long moved() {
        if (smEndPosition < smStartPosition) {
            return(smStartPosition - currentPosition);
        } else {
            return(currentPosition - smStartPosition);

        }
    }

    long distanceLeft() {
        return smDistance - moved();
    }

    public int smGetMovementError() {
        return (int) Math.abs(distanceLeft());
    }

    long distancePerTick(double velocity) {
        return (long) (velocity * tickSeconds());
    }
    long movedPerTick(double velocity) {
        return Math.abs(distancePerTick(velocity));
    }

    double tickSeconds() {
        return (tickTime / 1000.0);
    }
}




