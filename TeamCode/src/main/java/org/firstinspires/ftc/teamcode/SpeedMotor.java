package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;


public class SpeedMotor {
    static final int HISTORY_SIZE = 11;
    static final float NANOSECONDS_PER_SECOND = 1000000000;
    static final double LOOK_AHEAD_TIME = 0.05;
    static final double FINE_POWER_SCALE =  0.000005;
    static final double ROUGH_POAWER_SCALE = 0.00005;

    int position = 0;
    double desiredSpeed = 0;
    double currentSpeed = 0;
    double power = 0;
    double accelleration = 0;
    float updatesPerSecond = 0;
    double speedSoon = 0 ;
    double change = 0;
    double maxSpeed = 0;
    DcMotor motor = null;

    ArrayList<Integer> positionList = new ArrayList<Integer>();
    ArrayList<Long> timeList = new ArrayList<Long>();

    public SpeedMotor(DcMotor setMotor) {
        motor = setMotor;
    }

    public SpeedMotor(HardwareMap hardwareMap, String motorString, double maxSpeedIn) {
        motor =  hardwareMap.get(DcMotor.class, motorString);
        maxSpeed = maxSpeedIn;

    }

    public void setDirection(DcMotor.Direction direction ) {
        motor.setDirection(direction);
    }

    public SpeedMotor(DcMotor setMotor, double maxSpeedIn) {
        motor = setMotor;
        maxSpeed = maxSpeedIn;

    }

    public void setMaxSpeed(double maxSpeed) {
        this.maxSpeed = maxSpeed;
    }

    public void setPower(double power) {
        if (power > 1) {power = 1;}
        if (power < -1) { power = -1;}
        desiredSpeed = maxSpeed * power;
        autoAdjust();
    }

    public void setSpeed(double newDesiredSpeed) {
        desiredSpeed = newDesiredSpeed;
        autoAdjust();
    }

    public void updateCurrentSpeed() {
        position = motor.getCurrentPosition();
        long nanotime = System.nanoTime();
        positionList.add(position);
        timeList.add(nanotime);

        if (positionList.size() > HISTORY_SIZE) {
            positionList.remove(0);
        }
        if (timeList.size() > HISTORY_SIZE) {
            timeList.remove(0);
        }
        //set currentSpeed to
        if (timeList.size() > 3) {
            int start = 0;
            int end = timeList.size()-1;
            int middle = end/2;

            long elapsed =  timeList.get(end) - timeList.get(start);
            int distance = positionList.get(end) - positionList.get(start);
            long oldDistance = positionList.get(middle) - positionList.get(start);
            long newDistance = positionList.get(end) - positionList.get(middle);
            long oldElapsed = timeList.get(middle) - timeList.get(start);
            long newElapsed = timeList.get(end) - timeList.get(middle);
            if ((elapsed > 0 ) && (oldElapsed > 0) && (newElapsed > 0)) {
                currentSpeed = distance * (NANOSECONDS_PER_SECOND / elapsed);
                updatesPerSecond = (timeList.size() * (NANOSECONDS_PER_SECOND/elapsed));
                double oldSpeed = oldDistance * (NANOSECONDS_PER_SECOND / oldElapsed);
                double newSpeed = newDistance  * (NANOSECONDS_PER_SECOND / newElapsed);
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
        updateCurrentSpeed();
        speedSoon = currentSpeed + (accelleration * LOOK_AHEAD_TIME);
        if (speedSoon < desiredSpeed - 200) {
           //change = 0.02;
           change = -ROUGH_POAWER_SCALE * ( speedSoon - desiredSpeed );

        } else if (speedSoon > desiredSpeed + 200) {
            //change = -0.02;
            change = -ROUGH_POAWER_SCALE * ( speedSoon - desiredSpeed );
        } else {
            change = -FINE_POWER_SCALE * ( speedSoon - desiredSpeed );
            //power = speedSoon / desiredSpeed * power;
        }
        power=power+change;
        if (power > 1) power = 1;
        if (power < -1) power = -1;
        motor.setPower(power);

    }

    public double getPower() { return power; }
    public double getCurrentSpeed() { return currentSpeed; }
    public double getDesiredSpeed() { return desiredSpeed;}
    public double getAccelleration()  { return accelleration; }
    public double getSpeedSoon()  { return speedSoon; }


    public double getUpdatesPerSecond() {
        return updatesPerSecond;
    }

    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }



}
