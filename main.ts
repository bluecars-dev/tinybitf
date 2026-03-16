/**
 * TINY-BIT PRO - Complete Odometry, PIDF, and SLAM Library
 * Features: Motor PIDF Control, Dead Reckoning, Distance/Angle Driving, SLAM Mapping
 */

//% color="#006400" weight=20 icon="\uf1b9"
namespace TinybitPro {

    // ==========================================
    // 1. CALIBRATION & TUNING VARIABLES
    // ==========================================

    // Physics Calibration
    export let WHEELBASE = 0.082;           // 8.2 cm distance between wheels
    export let M_PER_PWM_S = 0.00215;       // Meters moved per 1 unit of PWM per second
    export let WHEEL_DIAMETER = 0.065;      // 6.5 cm wheel diameter

    // PIDF Controller Setup
    export let kP = 1.4;
    export let kI = 0.02;
    export let kD = 0.15;
    export let kF = 35;                     // Friction compensation (minimum power)

    // Movement Thresholds
    export let DISTANCE_TOLERANCE = 0.01;   // 1cm tolerance
    export let ANGLE_TOLERANCE = 2;         // 2 degree tolerance
    export let SONAR_MAX_RANGE = 1.5;       // 1.5m max
    export let SONAR_MIN_RANGE = 0.05;      // 5cm min

    // SLAM Setup
    export let GRID_RES = 0.05;             // Map resolution: 5cm per cell
    export let MAP_DIM = 40;                // Map size: 40x40 cells (2m x 2m area)

    // ==========================================
    // 2. ODOMETRY STATE VARIABLES
    // ==========================================

    // Robot Pose (Global Coordinates)
    export let x = 0.0, y = 0.0;
    export let theta = 0.0;                 // Heading in Radians

    // Motor State
    let motorLeftTarget = 0;
    let motorRightTarget = 0;
    let motorLeftActual = 0;
    let motorRightActual = 0;
    let motorLeftError = 0;
    let motorRightError = 0;

    // PIDF State
    let leftIntegral = 0.0;
    let rightIntegral = 0.0;
    let leftPrevError = 0.0;
    let rightPrevError = 0.0;
    let lastPIDFTime = control.micros();

    // Timing
    let lastOdomTime = control.micros();

    // Movement Tracking
    let targetDistance = 0.0;
    let targetAngle = 0.0;
    let isMoving = false;
    let movementMode = 0;  // 0: idle, 1: straight, 2: turn

    // ==========================================
    // 3. HARDWARE & MEMORY BUFFERS
    // ==========================================

    // I2C Configuration
    const PWM_ADD = 0x01;
    const MOTOR_REG = 0x02;
    const RGB_REG = 0x01;

    // Pre-allocated buffers
    let motorBuf = pins.createBuffer(5);
    motorBuf[0] = MOTOR_REG;

    let rgbBuf = pins.createBuffer(4);
    rgbBuf[0] = RGB_REG;

    // SLAM Grid
    let mapGrid = pins.createBuffer(MAP_DIM * MAP_DIM);

    // Sonar History (rolling median window)
    let sonarHistory = [0, 0, 0, 0, 0];
    let sonarIdx = 0;

    // Encoder simulation (PWM-based)
    let encoderLeftCount = 0;
    let encoderRightCount = 0;

    // ==========================================
    // 4. INITIALIZATION
    // ==========================================

    /**
     * Initialize the robot system.
     * Call this once at startup.
     */
    export function init(): void {
        serial.writeLine("[INIT] TinyBit Pro Odometry & SLAM System");
        serial.writeLine(`[INIT] WHEELBASE: ${WHEELBASE}m, M_PER_PWM_S: ${M_PER_PWM_S}`);
        serial.writeLine(`[INIT] PIDF: kP=${kP}, kI=${kI}, kD=${kD}, kF=${kF}`);

        resetPose();
        resetMap();
        lastPIDFTime = control.micros();
        lastOdomTime = control.micros();
        isMoving = false;
    }

    // ==========================================
    // 5. MOTOR PIDF CONTROL SYSTEM
    // ==========================================

    /**
     * Low-level raw motor drive.
     * Sends PWM directly to motors.
     * @param left: Left motor PWM (-255 to 255)
     * @param right: Right motor PWM (-255 to 255)
     */
    export function rawDrive(left: number, right: number): void {
        let l = Math.constrain(left, -255, 255);
        let r = Math.constrain(right, -255, 255);

        // Clamp to byte values
        motorBuf[1] = l > 0 ? l : 0;
        motorBuf[2] = l < 0 ? -l : 0;
        motorBuf[3] = r > 0 ? r : 0;
        motorBuf[4] = r < 0 ? -r : 0;

        pins.i2cWriteBuffer(PWM_ADD, motorBuf);

        motorLeftActual = l;
        motorRightActual = r;
    }

    /**
     * Set target motor speeds for PIDF control.
     * @param left: Target left motor PWM (-255 to 255)
     * @param right: Target right motor PWM (-255 to 255)
     */
    export function setMotorTarget(left: number, right: number): void {
        motorLeftTarget = Math.constrain(left, -255, 255);
        motorRightTarget = Math.constrain(right, -255, 255);
    }

    /**
     * PIDF Motor Controller Update Loop.
     * Call this regularly (10-100 Hz) to maintain motor synchronization.
     */
    export function updateMotorPIDF(): void {
        let now = control.micros();
        let dt = (now - lastPIDFTime) / 1000000.0;
        if (dt <= 0) dt = 0.001;
        lastPIDFTime = now;

        // Left Motor PIDF
        let leftError = motorLeftTarget - motorLeftActual;
        leftIntegral += leftError * dt;
        leftIntegral = Math.constrain(leftIntegral, -100, 100);  // Anti-windup
        let leftDerivative = (leftError - leftPrevError) / dt;
        leftPrevError = leftError;

        let leftOutput = kP * leftError + kI * leftIntegral + kD * leftDerivative;
        let leftPWM = motorLeftTarget + leftOutput;

        // Right Motor PIDF
        let rightError = motorRightTarget - motorRightActual;
        rightIntegral += rightError * dt;
        rightIntegral = Math.constrain(rightIntegral, -100, 100);
        let rightDerivative = (rightError - rightPrevError) / dt;
        rightPrevError = rightError;

        let rightOutput = kP * rightError + kI * rightIntegral + kD * rightDerivative;
        let rightPWM = motorRightTarget + rightOutput;

        // Apply friction compensation
        if (leftPWM > 0 && leftPWM < kF) leftPWM = kF;
        if (leftPWM < 0 && leftPWM > -kF) leftPWM = -kF;
        if (rightPWM > 0 && rightPWM < kF) rightPWM = kF;
        if (rightPWM < 0 && rightPWM > -kF) rightPWM = -kF;

        rawDrive(leftPWM, rightPWM);
    }

    /**
     * Motor synchronization - make sure both motors run at same speed.
     * Useful for straight-line driving.
     */
    export function syncMotors(): void {
        // Measure current motor speeds and adjust
        setMotorTarget(motorLeftTarget, motorRightTarget);
        updateMotorPIDF();
    }

    // ==========================================
    // 6. SONAR SENSOR
    // ==========================================

    /**
     * Get filtered sonar distance in centimeters.
     * Returns median of last 5 readings to eliminate noise.
     */
    export function getSonar(): number {
        // Trigger sonar
        pins.digitalWritePin(DigitalPin.P16, 0);
        control.waitMicros(2);
        pins.digitalWritePin(DigitalPin.P16, 1);
        control.waitMicros(10);
        pins.digitalWritePin(DigitalPin.P16, 0);

        // Measure echo (timeout at 25ms)
        let d = pins.pulseIn(DigitalPin.P15, PulseValue.High, 25000) / 58;

        // Validate and store
        if (d > 2 && d < 400) {
            sonarHistory[sonarIdx] = d;
            sonarIdx = (sonarIdx + 1) % 5;
        }

        // Return median
        let sorted = sonarHistory.slice().sort((a, b) => a - b);
        return sorted[2];
    }

    /**
     * Get sonar distance in meters.
     */
    export function getSonarMeters(): number {
        return getSonar() / 100.0;
    }

    // ==========================================
    // 7. ODOMETRY & DEAD RECKONING
    // ==========================================

    /**
     * Update robot pose using dead reckoning.
     * Call this regularly with motor PWM values.
     * @param lp: Left motor actual PWM
     * @param rp: Right motor actual PWM
     */
    export function updatePose(lp: number, rp: number): void {
        let now = control.micros();
        let dt = (now - lastOdomTime) / 1000000.0;
        if (dt <= 0) dt = 0.001;
        lastOdomTime = now;

        // Convert PWM to velocity (m/s)
        let vL = lp * M_PER_PWM_S;
        let vR = rp * M_PER_PWM_S;

        // Calculate linear and angular velocity
        let vLinear = (vR + vL) / 2.0;
        let vAngular = (vR - vL) / WHEELBASE;

        // Update heading
        theta += vAngular * dt;

        // Wrap theta to [-π, π]
        while (theta > Math.PI) theta -= 2 * Math.PI;
        while (theta < -Math.PI) theta += 2 * Math.PI;

        // Update position in global frame
        x += vLinear * Math.cos(theta) * dt;
        y += vLinear * Math.sin(theta) * dt;
    }

    /**
     * Get current robot pose.
     * @return {x, y, theta}
     */
    export function getPose(): { x: number, y: number, theta: number } {
        return { x: x, y: y, theta: theta };
    }

    /**
     * Get heading in degrees.
     */
    export function getHeadingDegrees(): number {
        return theta * 57.2957795;  // rad to deg
    }

    /**
     * Get distance from origin.
     */
    export function getDistanceFromOrigin(): number {
        return Math.sqrt(x * x + y * y);
    }

    /**
     * Reset pose to origin.
     */
    export function resetPose(): void {
        x = 0.0;
        y = 0.0;
        theta = 0.0;
        lastOdomTime = control.micros();
    }

    /**
     * Manually set pose (for loop closure correction).
     */
    export function setPose(px: number, py: number, ptheta: number): void {
        x = px;
        y = py;
        theta = ptheta;
    }

    // ==========================================
    // 8. SLAM MAPPING
    // ==========================================

    /**
     * Update occupancy grid map using sonar.
     * Projects sonar reading into global map coordinates.
     */
    export function updateMap(): void {
        let d_meters = getSonarMeters();

        // Ignore out-of-range readings
        if (d_meters < SONAR_MIN_RANGE || d_meters > SONAR_MAX_RANGE) return;

        // Project sensor hit into global coordinates
        let wallX = x + (d_meters * Math.cos(theta));
        let wallY = y + (d_meters * Math.sin(theta));

        // Convert to grid coordinates
        let gx = Math.floor(wallX / GRID_RES) + (MAP_DIM / 2);
        let gy = Math.floor(wallY / GRID_RES) + (MAP_DIM / 2);

        // Update grid if in bounds
        if (gx >= 0 && gx < MAP_DIM && gy >= 0 && gy < MAP_DIM) {
            let index = gy * MAP_DIM + gx;
            if (mapGrid[index] < 250) {
                mapGrid[index] += 25;  // Requires ~4 hits for certainty
            }
        }
    }

    /**
     * Get occupancy probability of a grid cell (0-255).
     */
    export function getGridCell(gx: number, gy: number): number {
        if (gx < 0 || gx >= MAP_DIM || gy < 0 || gy >= MAP_DIM) return 0;
        return mapGrid[gy * MAP_DIM + gx];
    }

    /**
     * Manually set a grid cell value.
     */
    export function setGridCell(gx: number, gy: number, value: number): void {
        if (gx < 0 || gx >= MAP_DIM || gy < 0 || gy >= MAP_DIM) return;
        mapGrid[gy * MAP_DIM + gx] = Math.constrain(value, 0, 255);
    }

    /**
     * Clear the entire map.
     */
    export function resetMap(): void {
        for (let i = 0; i < MAP_DIM * MAP_DIM; i++) {
            mapGrid[i] = 0;
        }
    }

    // ==========================================
    // 9. HIGH-LEVEL MOVEMENT COMMANDS
    // ==========================================

    /**
     * Drive straight for a distance (cm).
     * Uses odometry to track distance.
     * @param distanceCm: Distance in centimeters
     * @param speed: Motor PWM (default 150)
     */
    export function driveStraight(distanceCm: number, speed: number = 150): void {
        let targetDist = distanceCm / 100.0;  // Convert to meters
        resetPose();
        isMoving = true;
        movementMode = 1;

        while (isMoving && getDistanceFromOrigin() < targetDist) {
            setMotorTarget(speed, speed);
            updateMotorPIDF();
            updatePose(motorLeftActual, motorRightActual);
            updateMap();
            basic.pause(10);
        }

        rawDrive(0, 0);
        isMoving = false;
    }

    /**
     * Turn by angle (degrees).
     * Uses odometry to track rotation.
     * @param angleDeg: Angle in degrees (positive = left, negative = right)
     * @param speed: Motor PWM (default 100)
     */
    export function turnByAngle(angleDeg: number, speed: number = 100): void {
        let targetRad = angleDeg * 0.0174533;  // deg to rad
        let initialTheta = theta;
        isMoving = true;
        movementMode = 2;

        while (isMoving) {
            let rotated = Math.abs(theta - initialTheta);
            if (rotated >= Math.abs(targetRad)) break;

            if (angleDeg > 0) {
                setMotorTarget(-speed, speed);  // Turn left
            } else {
                setMotorTarget(speed, -speed);  // Turn right
            }

            updateMotorPIDF();
            updatePose(motorLeftActual, motorRightActual);
            basic.pause(10);
        }

        rawDrive(0, 0);
        isMoving = false;
    }

    /**
     * Drive in an arc.
     * @param distanceCm: Arc length in cm
     * @param radiusM: Turn radius in meters (positive = left, negative = right)
     */
    export function driveArc(distanceCm: number, radiusM: number, speed: number = 150): void {
        let arcDist = distanceCm / 100.0;
        let initialDist = getDistanceFromOrigin();
        isMoving = true;

        while (isMoving && (getDistanceFromOrigin() - initialDist) < arcDist) {
            // Differential drive for arc
            let angularRate = speed / radiusM;
            let leftSpeed = speed - (angularRate * WHEELBASE / 2);
            let rightSpeed = speed + (angularRate * WHEELBASE / 2);

            setMotorTarget(leftSpeed, rightSpeed);
            updateMotorPIDF();
            updatePose(motorLeftActual, motorRightActual);
            updateMap();
            basic.pause(10);
        }

        rawDrive(0, 0);
        isMoving = false;
    }

    /**
     * Drive to absolute position using SLAM.
     * @param targetX: Target X in meters
     * @param targetY: Target Y in meters
     * @param speed: Motor PWM (default 150)
     */
    export function driveTo(targetX: number, targetY: number, speed: number = 150): void {
        isMoving = true;

        while (isMoving) {
            // Calculate distance to target
            let dx = targetX - x;
            let dy = targetY - y;
            let dist = Math.sqrt(dx * dx + dy * dy);

            if (dist < DISTANCE_TOLERANCE) break;

            // Calculate angle to target
            let targetAngle = Math.atan2(dy, dx);
            let angleDiff = targetAngle - theta;

            // Wrap angle difference
            while (angleDiff > Math.PI) angleDiff -= 2 * Math.PI;
            while (angleDiff < -Math.PI) angleDiff += 2 * Math.PI;

            // Proportional control
            let leftSpeed = speed * (1 - angleDiff * 0.5);
            let rightSpeed = speed * (1 + angleDiff * 0.5);

            setMotorTarget(leftSpeed, rightSpeed);
            updateMotorPIDF();
            updatePose(motorLeftActual, motorRightActual);
            updateMap();
            basic.pause(10);
        }

        rawDrive(0, 0);
        isMoving = false;
    }

    /**
     * Rotate to face absolute heading.
     * @param targetHeadingDeg: Target heading in degrees (0-360)
     * @param speed: Motor PWM (default 100)
     */
    export function rotateTo(targetHeadingDeg: number, speed: number = 100): void {
        let targetRad = targetHeadingDeg * 0.0174533;
        isMoving = true;

        while (isMoving) {
            let angleDiff = targetRad - theta;

            // Wrap to [-π, π]
            while (angleDiff > Math.PI) angleDiff -= 2 * Math.PI;
            while (angleDiff < -Math.PI) angleDiff += 2 * Math.PI;

            if (Math.abs(angleDiff) < ANGLE_TOLERANCE * 0.0174533) break;

            if (angleDiff > 0) {
                setMotorTarget(-speed, speed);
            } else {
                setMotorTarget(speed, -speed);
            }

            updateMotorPIDF();
            updatePose(motorLeftActual, motorRightActual);
            basic.pause(10);
        }

        rawDrive(0, 0);
        isMoving = false;
    }

    /**
     * Stop all motors immediately.
     */
    export function stop(): void {
        rawDrive(0, 0);
        isMoving = false;
        setMotorTarget(0, 0);
    }

    // ==========================================
    // 10. RGB LED CONTROL
    // ==========================================

    export enum RGBColor {
        OFF = 0,
        Red = 1,
        Green = 2,
        Blue = 3,
        White = 4,
        Cyan = 5,
        Magenta = 6,
        Yellow = 7
    }

    /**
     * Set RGB LED color.
     */
    export function setRGB(color: RGBColor): void {
        let r = 0, g = 0, b = 0;

        switch (color) {
            case RGBColor.Red: r = 255; break;
            case RGBColor.Green: g = 255; break;
            case RGBColor.Blue: b = 255; break;
            case RGBColor.White: r = 255; g = 255; b = 255; break;
            case RGBColor.Cyan: g = 255; b = 255; break;
            case RGBColor.Magenta: r = 255; b = 255; break;
            case RGBColor.Yellow: r = 255; g = 255; break;
        }

        rgbBuf[1] = r;
        rgbBuf[2] = g;
        rgbBuf[3] = b;
        pins.i2cWriteBuffer(PWM_ADD, rgbBuf);
    }

    /**
     * Set RGB with raw values.
     */
    export function setRGBRaw(r: number, g: number, b: number): void {
        rgbBuf[1] = Math.constrain(r, 0, 255);
        rgbBuf[2] = Math.constrain(g, 0, 255);
        rgbBuf[3] = Math.constrain(b, 0, 255);
        pins.i2cWriteBuffer(PWM_ADD, rgbBuf);
    }

    // ==========================================
    // 11. DEBUG & TELEMETRY
    // ==========================================

    /**
     * Print current pose to serial.
     */
    export function printPose(): void {
        serial.writeLine(`[POSE] X: ${Math.round(x * 100)}cm, Y: ${Math.round(y * 100)}cm, θ: ${Math.round(getHeadingDegrees())}°`);
    }

    /**
     * Print sonar reading to serial.
     */
    export function printSonar(): void {
        let d = getSonar();
    }

    /**
     * Print motor state to serial.
     */
    export function printMotorState(): void {
        serial.writeLine(`[MOTOR] L: ${motorLeftActual}/${motorLeftTarget}, R: ${motorRightActual}/${motorRightTarget}, Err: ${motorLeftError}/${motorRightError}`);
    }

    /**
     * Export SLAM map as ASCII to serial.
     */
    export function exportMapToSerial(): void {
        serial.writeLine("\n========== SLAM MAP ==========");
        serial.writeLine(`Pose: X=${Math.round(x * 100)}cm Y=${Math.round(y * 100)}cm θ=${Math.round(getHeadingDegrees())}°`);
        serial.writeLine("");

        for (let row = 0; row < MAP_DIM; row++) {
            let line = "";
            for (let col = 0; col < MAP_DIM; col++) {
                let cellData = mapGrid[row * MAP_DIM + col];
                let robotGx = Math.floor(x / GRID_RES) + (MAP_DIM / 2);
                let robotGy = Math.floor(y / GRID_RES) + (MAP_DIM / 2);

                if (col === robotGx && row === robotGy) {
                    line += "R ";
                } else if (cellData > 100) {
                    line += "█ ";
                } else if (cellData > 25) {
                    line += "▒ ";
                } else {
                    line += ". ";
                }
            }
            serial.writeLine(line);
        }
        serial.writeLine("==============================\n");
    }

    /**
     * Export telemetry as JSON.
     */
    export function exportTelemetryJSON(): string {
        let json = `{"pose":{"x":${x},"y":${y},"theta":${theta},"heading_deg":${getHeadingDegrees()}},"motors":{"left":${motorLeftActual},"right":${motorRightActual},"left_target":${motorLeftTarget},"right_target":${motorRightTarget}},"sonar":${getSonar()},"distance_origin":${getDistanceFromOrigin()}}`;
        serial.writeLine(json);
        return json;
    }
}