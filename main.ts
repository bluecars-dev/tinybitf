namespace TinybitPro {
    // ==========================================
    // 1. CALIBRATION & TUNING
    // ==========================================
    export let WHEELBASE = 0.045;
    export let M_PER_PWM_S = 0.0086;
    export let kF = 85;               // High enough to break static friction
    export let SLEW_RATE = 15;        // Smooths acceleration automatically

    export let x = 0.0, y = 0.0, theta = 0.0;
    export let motorLeftTarget = 0, motorRightTarget = 0;
    export let motorLeftActual = 0, motorRightActual = 0;

    let lastOdomTime = control.micros();
    export let isMoving = false;

    const PWM_ADD = 0x01;
    const MOTOR_REG = 0x02;
    let motorBuf = pins.createBuffer(5);
    motorBuf[0] = MOTOR_REG;

    // ==========================================
    // 2. AUTOMATIC HEARTBEAT (The "Brain")
    // ==========================================
    // This runs in the background every 20ms automatically.
    control.inBackground(function () {
        while (true) {
            updatePose();        // Math: Tracks where we are
            updateMotorLogic();  // Hardware: Ramps motors to targets
            basic.pause(20);     // 50Hz Refresh Rate
        }
    })

    function updateMotorLogic(): void {
        let lDiff = motorLeftTarget - motorLeftActual;
        let rDiff = motorRightTarget - motorRightActual;

        // Apply Slew Rate (Ramping)
        if (Math.abs(lDiff) > SLEW_RATE) motorLeftActual += (lDiff > 0 ? SLEW_RATE : -SLEW_RATE);
        else motorLeftActual = motorLeftTarget;

        if (Math.abs(rDiff) > SLEW_RATE) motorRightActual += (rDiff > 0 ? SLEW_RATE : -SLEW_RATE);
        else motorRightActual = motorRightTarget;

        rawDrive(motorLeftActual, motorRightActual);
    }

    function updatePose(): void {
        let now = control.micros();
        let dt = (now - lastOdomTime) / 1000000.0;
        if (dt <= 0) dt = 0.001;
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

    // ==========================================
    // 3. LOW-LEVEL I2C (Fixed for Backwards)
    // ==========================================
    export function rawDrive(left: number, right: number): void {
        let l = Math.constrain(Math.round(left), -255, 255);
        let r = Math.constrain(Math.round(right), -255, 255);

        // Kickstart logic
        if (l != 0 && Math.abs(l) < kF) l = (l > 0) ? kF : -kF;
        if (r != 0 && Math.abs(r) < kF) r = (r > 0) ? kF : -r;

        // FIX: Clear buffer so Forward and Backward pins don't conflict
        motorBuf[1] = 0; motorBuf[2] = 0; motorBuf[3] = 0; motorBuf[4] = 0;

        if (l > 0) motorBuf[1] = l;
        else if (l < 0) motorBuf[2] = -l;

        if (r > 0) motorBuf[3] = r;
        else if (r < 0) motorBuf[4] = -r;

        pins.i2cWriteBuffer(PWM_ADD, motorBuf);
    }

    // ==========================================
    // 4. FIXED NAVIGATION COMMANDS
    // ==========================================
    export function driveStraight(distanceCm: number, speed: number = 160): void {
        let targetM = Math.abs(distanceCm / 100.0);
        let startX = x, startY = y;
        let dir = (distanceCm >= 0) ? 1 : -1;

        motorLeftTarget = speed * dir;
        motorRightTarget = speed * dir;
        isMoving = true;

        while (isMoving) {
            let traveled = Math.sqrt(Math.pow(x - startX, 2) + Math.pow(y - startY, 2));
            if (traveled >= targetM) isMoving = false;
            basic.pause(5);
        }
        stop();
    }

    export function turnByAngle(angleDeg: number, speed: number = 180): void {
        let targetRad = Math.abs(angleDeg * (Math.PI / 180));
        let startTheta = theta;
        let dir = (angleDeg > 0) ? 1 : -1;

        motorLeftTarget = -speed * dir;
        motorRightTarget = speed * dir;
        isMoving = true;

        while (isMoving) {
            let diff = Math.abs(theta - startTheta);
            if (diff > Math.PI) diff = 2 * Math.PI - diff;
            if (diff >= targetRad) isMoving = false;
            basic.pause(5);
        }
        stop();
    }

    export function stop(): void {
        motorLeftTarget = 0;
        motorRightTarget = 0;
        isMoving = false;
    }

    export function init(): void {
        x = 0; y = 0; theta = 0;
        lastOdomTime = control.micros();
        stop();
    }
}


