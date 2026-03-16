/**
 * TINY-BIT PRO - Gearbox & SLAM Optimized Edition
 * Fixes: Gearbox stall, I2C oscillation, and phantom PID jitter.
 */

//% color="#006400" weight=20 icon="\uf1b9"
namespace TinybitPro {

    // ==========================================
    // 1. CALIBRATION & TUNING
    // ==========================================
    export let WHEELBASE = 0.082;         // 8.2cm
    export let M_PER_PWM_S = 0.00215;    // Meters per PWM unit per sec
    
    // GEARBOX TUNING
    export let kF = 60;                  // Minimum "Kick" to move gears (45-70)
    export let SLEW_RATE = 15;           // Max PWM change per 20ms
    export let DISTANCE_TOLERANCE = 0.03; // 3cm
    export let ANGLE_TOLERANCE = 0.05;    // Radians (~3 degrees)

    // SLAM SETUP
    export let GRID_RES = 0.05; 
    export let MAP_DIM = 40; 

    // ==========================================
    // 2. INTERNAL STATE
    // ==========================================
    export let x = 0.0, y = 0.0, theta = 0.0;
    let motorLeftTarget = 0, motorRightTarget = 0;
    let motorLeftActual = 0, motorRightActual = 0;
    
    let lastOdomTime = control.micros();
    let isMoving = false;

    // Directional safety (prevents gear grinding/oscillation)
    let car_flag_old = 0; 
    let car_flag_new = 0; 

    const PWM_ADD = 0x01;
    const MOTOR_REG = 0x02;
    const RGB_REG = 0x01;

    let motorBuf = pins.createBuffer(5);
    motorBuf[0] = MOTOR_REG;
    let mapGrid = pins.createBuffer(MAP_DIM * MAP_DIM);

    // ==========================================
    // 3. LOW-LEVEL HARDWARE CONTROL
    // ==========================================

    /**
     * Sends raw PWM to the I2C controller with Gearbox Safety.
     */
    export function rawDrive(left: number, right: number): void {
        let l = Math.constrain(Math.round(left), -255, 255);
        let r = Math.constrain(Math.round(right), -255, 255);

        // Gearbox Logic: Boost small signals to kF so gears actually turn
        if (l != 0 && Math.abs(l) < kF) l = (l > 0) ? kF : -kF;
        if (r != 0 && Math.abs(r) < kF) r = (r > 0) ? kF : -r;
        
        // Zero-out noise
        if (Math.abs(l) < 10) l = 0;
        if (Math.abs(r) < 10) r = 0;

        // Directional State Tracking
        if (l >= 0 && r >= 0) car_flag_new = 0;      
        else if (l < 0 && r < 0) car_flag_new = 1;  
        else if (l > 0 && r < 0) car_flag_new = 2;  
        else car_flag_new = 3;                      

        // SAFETY: If direction flipped, stop briefly to protect gearbox
        if (car_flag_new != car_flag_old) {
            let stopBuf = pins.createBuffer(5);
            stopBuf[0] = MOTOR_REG; // All zeros
            pins.i2cWriteBuffer(PWM_ADD, stopBuf);
            basic.pause(100); 
            car_flag_old = car_flag_new;
        }

        motorBuf[1] = l > 0 ? l : 0;
        motorBuf[2] = l < 0 ? -l : 0;
        motorBuf[3] = r > 0 ? r : 0;
        motorBuf[4] = r < 0 ? -r : 0;
        
        pins.i2cWriteBuffer(PWM_ADD, motorBuf);
        motorLeftActual = l;
        motorRightActual = r;
    }

    /**
     * Ramps motor speed to target to prevent high-frequency oscillation.
     */
    export function updateMotorLogic(): void {
        let lDiff = motorLeftTarget - motorLeftActual;
        let rDiff = motorRightTarget - motorRightActual;

        // Apply Slew Rate (Smoothing)
        if (Math.abs(lDiff) > SLEW_RATE) motorLeftActual += (lDiff > 0 ? SLEW_RATE : -SLEW_RATE);
        else motorLeftActual = motorLeftTarget;

        if (Math.abs(rDiff) > SLEW_RATE) motorRightActual += (rDiff > 0 ? SLEW_RATE : -SLEW_RATE);
        else motorRightActual = motorRightTarget;

        rawDrive(motorLeftActual, motorRightActual);
    }

    // ==========================================
    // 4. ODOMETRY & SLAM
    // ==========================================

    export function updatePose(): void {
        let now = control.micros();
        let dt = (now - lastOdomTime) / 1000000.0;
        if (dt <= 0) dt = 0.001;
        lastOdomTime = now;

        let vL = motorLeftActual * M_PER_PWM_S;
        let vR = motorRightActual * M_PER_PWM_S;

        let vLinear = (vR + vL) / 2.0;
        let vAngular = (vR - vL) / WHEELBASE;

        theta += vAngular * dt;
        // Normalize Angle
        while (theta > Math.PI) theta -= 2 * Math.PI;
        while (theta < -Math.PI) theta += 2 * Math.PI;

        x += vLinear * Math.cos(theta) * dt;
        y += vLinear * Math.sin(theta) * dt;
    }

    export function getSonar(): number {
        pins.digitalWritePin(DigitalPin.P16, 0);
        control.waitMicros(2);
        pins.digitalWritePin(DigitalPin.P16, 1);
        control.waitMicros(10);
        pins.digitalWritePin(DigitalPin.P16, 0);
        let d = pins.pulseIn(DigitalPin.P15, PulseValue.High, 25000);
        return d / 58;
    }

    // ==========================================
    // 5. NAVIGATION COMMANDS
    // ==========================================

    export function driveStraight(distanceCm: number, speed: number = 150): void {
        let targetDist = distanceCm / 100.0;
        let startX = x, startY = y;
        isMoving = true;

        while (isMoving) {
            updatePose();
            let traveled = Math.sqrt(Math.pow(x - startX, 2) + Math.pow(y - startY, 2));
            if (traveled >= targetDist) break;

            motorLeftTarget = speed;
            motorRightTarget = speed;
            updateMotorLogic();
            basic.pause(20); 
        }
        stop();
    }

    export function turnByAngle(angleDeg: number, speed: number = 120): void {
        let targetRad = angleDeg * (Math.PI / 180);
        let startTheta = theta;
        isMoving = true;

        while (isMoving) {
            updatePose();
            let diff = Math.abs(theta - startTheta);
            if (diff >= Math.abs(targetRad)) break;

            if (angleDeg > 0) {
                motorLeftTarget = -speed; motorRightTarget = speed;
            } else {
                motorLeftTarget = speed; motorRightTarget = -speed;
            }
            updateMotorLogic();
            basic.pause(20);
        }
        stop();
    }

    export function stop(): void {
        motorLeftTarget = 0;
        motorRightTarget = 0;
        rawDrive(0, 0);
        isMoving = false;
    }

    export function init(): void {
        x = 0; y = 0; theta = 0;
        lastOdomTime = control.micros();
        stop();
        for (let i = 0; i < MAP_DIM * MAP_DIM; i++) mapGrid[i] = 0;
    }
}
