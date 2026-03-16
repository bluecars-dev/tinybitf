/**
 * TINY-BIT PRO - Stabilized Edition
 * Fixed: Removed phantom PID oscillation, added Slew-Rate control
 */

//% color="#006400" weight=20 icon="\uf1b9"
namespace TinybitPro {

    // ==========================================
    // 1. CALIBRATION & TUNING VARIABLES (FIXED)
    // ==========================================
    export let WHEELBASE = 0.082;
    export let M_PER_PWM_S = 0.00215;
    
    // TUNING PARAMETERS:
    export let kF = 35;           // Min power to move (Friction)
    export let SLEW_RATE = 10;    // How fast PWM changes per 10ms (Lower = smoother)
    export let DISTANCE_TOLERANCE = 0.03; // 3cm tolerance to prevent "hunting"
    export let ANGLE_TOLERANCE = 3;       // 3 degree tolerance

    // SLAM/Map
    export let GRID_RES = 0.05;
    export let MAP_DIM = 40;

    // ==========================================
    // 2. STATE VARIABLES
    // ==========================================
    export let x = 0.0, y = 0.0, theta = 0.0;
    let motorLeftTarget = 0, motorRightTarget = 0;
    let motorLeftActual = 0, motorRightActual = 0;
    let lastOdomTime = control.micros();
    let isMoving = false;

    const PWM_ADD = 0x01;
    const MOTOR_REG = 0x02;
    let motorBuf = pins.createBuffer(5);
    motorBuf[0] = MOTOR_REG;
    let mapGrid = pins.createBuffer(MAP_DIM * MAP_DIM);

    // ==========================================
    // 3. CORE CONTROL (STABILIZED)
    // ==========================================

    export function rawDrive(left: number, right: number): void {
        let l = Math.constrain(left, -255, 255);
        let r = Math.constrain(right, -255, 255);

        // Apply Friction Deadzone: If power is too low to move, snap to kF or 0
        if (Math.abs(l) > 0 && Math.abs(l) < kF) l = (l > 0) ? kF : -kF;
        if (Math.abs(r) > 0 && Math.abs(r) < kF) r = (r > 0) ? kF : -r;
        if (Math.abs(l) < 5) l = 0; 
        if (Math.abs(r) < 5) r = 0;

        motorBuf[1] = l > 0 ? l : 0;
        motorBuf[2] = l < 0 ? -l : 0;
        motorBuf[3] = r > 0 ? r : 0;
        motorBuf[4] = r < 0 ? -r : 0;

        pins.i2cWriteBuffer(PWM_ADD, motorBuf);
        motorLeftActual = l;
        motorRightActual = r;
    }

    /**
     * Replaces PID. Smoothly ramps motor power to target.
     * This eliminates the rapid "jitter" or "oscillation".
     */
    export function updateMotorLogic(): void {
        // Ramp Left Motor
        let leftDiff = motorLeftTarget - motorLeftActual;
        if (Math.abs(leftDiff) > SLEW_RATE) {
            motorLeftActual += (leftDiff > 0 ? SLEW_RATE : -SLEW_RATE);
        } else {
            motorLeftActual = motorLeftTarget;
        }

        // Ramp Right Motor
        let rightDiff = motorRightTarget - motorRightActual;
        if (Math.abs(rightDiff) > SLEW_RATE) {
            motorRightActual += (rightDiff > 0 ? SLEW_RATE : -SLEW_RATE);
        } else {
            motorRightActual = motorRightTarget;
        }

        rawDrive(motorLeftActual, motorRightActual);
    }

    export function setMotorTarget(left: number, right: number): void {
        motorLeftTarget = left;
        motorRightTarget = right;
    }

    // ==========================================
    // 4. MOVEMENT & ODOMETRY
    // ==========================================

    export function updatePose(): void {
        let now = control.micros();
        let dt = (now - lastOdomTime) / 1000000.0;
        lastOdomTime = now;

        let vL = motorLeftActual * M_PER_PWM_S;
        let vR = motorRightActual * M_PER_PWM_S;
        let vLinear = (vR + vL) / 2.0;
        let vAngular = (vR - vL) / WHEELBASE;

        theta += vAngular * dt;
        while (theta > Math.PI) theta -= 2 * Math.PI;
        while (theta < -Math.PI) theta += 2 * Math.PI;

        x += vLinear * Math.cos(theta) * dt;
        y += vLinear * Math.sin(theta) * dt;
    }

    export function driveStraight(distanceCm: number, speed: number = 150): void {
        let targetDist = distanceCm / 100.0;
        let startX = x, startY = y;
        isMoving = true;

        while (isMoving) {
            let traveled = Math.sqrt(Math.pow(x - startX, 2) + Math.pow(y - startY, 2));
            if (traveled >= targetDist) break;

            setMotorTarget(speed, speed);
            updateMotorLogic();
            updatePose();
            basic.pause(20); // 20ms pause helps I2C stability
        }
        stop();
    }

    export function stop(): void {
        setMotorTarget(0, 0);
        rawDrive(0, 0);
        isMoving = false;
    }

    export function init(): void {
        x = 0; y = 0; theta = 0;
        lastOdomTime = control.micros();
        stop();
    }
}
