package org.firstinspires.ftc.teamcode.opmodes

import com.amarcolini.joos.control.FeedforwardCoefficients
import com.amarcolini.joos.hardware.Motor
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Drive.DRIVE_LEFT_A_NAME
import org.firstinspires.ftc.teamcode.Drive.DRIVE_LEFT_B_NAME

import org.firstinspires.ftc.teamcode.Drive.GEAR_RATIO
import org.firstinspires.ftc.teamcode.Drive.WHEEL_RADIUS
import org.firstinspires.ftc.teamcode.Motors.ULTRAPLANETARY_MAX_RPM
import org.firstinspires.ftc.teamcode.Motors.ULTRAPLANETARY_TICKS
import kotlin.math.PI

@TeleOp(name = "Motor Testing", group = "Testing")
class MotorTests : OpMode() {
    // this is all you need to do for an op mode.

    private lateinit var motorA: Motor
    private lateinit var motorB: Motor
    override fun init() {
        // make sure to set it as the right TeleOp

        motorA = Motor(
            hardwareMap, DRIVE_LEFT_A_NAME,
            ULTRAPLANETARY_MAX_RPM,
            ULTRAPLANETARY_TICKS
        )

        motorB = Motor(
            hardwareMap, DRIVE_LEFT_B_NAME,
            ULTRAPLANETARY_MAX_RPM,
            ULTRAPLANETARY_TICKS
        )

        motorA.let {
            it.runMode = Motor.RunMode.RUN_WITHOUT_ENCODER
            it.distancePerRev = 2 * PI * WHEEL_RADIUS * GEAR_RATIO
            it.feedforwardCoefficients =
                FeedforwardCoefficients(1 / it.rpmToDistanceVelocity(ULTRAPLANETARY_MAX_RPM))
        }

        motorB.let {
            it.runMode = Motor.RunMode.RUN_WITHOUT_ENCODER
            it.distancePerRev = 2 * PI * WHEEL_RADIUS * GEAR_RATIO
            it.feedforwardCoefficients =
                FeedforwardCoefficients(1 / it.rpmToDistanceVelocity(ULTRAPLANETARY_MAX_RPM))
        }

        motorA.zeroPowerBehavior = Motor.ZeroPowerBehavior.FLOAT
        motorB.zeroPowerBehavior = Motor.ZeroPowerBehavior.FLOAT
    }

    override fun loop() {
        telemetry.addData("Motor A", motorA.currentPosition)
        telemetry.addData("Motor B", motorB.currentPosition)
        telemetry.update()
    }
}