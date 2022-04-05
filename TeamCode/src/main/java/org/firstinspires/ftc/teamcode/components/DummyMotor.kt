package org.firstinspires.ftc.teamcode.components

import com.amarcolini.joos.util.NanoClock
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.roundToInt

class DummyMotor(
    private val RPM: Double = 100.0,
    private val TPR: Double = 100.0,
    private val kV: Double = 1.0,
    private val kA: Double = Double.POSITIVE_INFINITY,
    private val kStatic: Double = 0.0,
    private val clock: NanoClock = NanoClock.system()
) : DcMotorEx {
    override fun getManufacturer(): HardwareDevice.Manufacturer = HardwareDevice.Manufacturer.Other
    override fun getDeviceName() = "dummy"
    override fun getConnectionInfo() = ""
    override fun getVersion() = -1

    private var reversed: Boolean = false
    private var power: Double = 0.0
    private var vel: Double = 0.0
    private var encoder: Int = 0

    private var last: Double? = null
    fun update(seconds: Double = clock.seconds()) {
        val lastUpdate = last
        if (lastUpdate == null) {
            last = seconds
            return
        }
        val dt = seconds - lastUpdate
        val target = power * RPM * TPR / 60 * kV + kStatic
        vel = target - (target - vel) / (abs(target - vel) * dt * kA + 1)
        encoder += (vel * dt).roundToInt()
        last = seconds
    }

    override fun resetDeviceConfigurationForOpMode() {
        TODO("Not yet implemented")
    }

    override fun close() {
        TODO("Not yet implemented")
    }

    override fun setDirection(direction: DcMotorSimple.Direction?) {
        reversed = direction == DcMotorSimple.Direction.REVERSE
    }

    override fun getDirection(): DcMotorSimple.Direction =
        if (reversed) DcMotorSimple.Direction.REVERSE
        else DcMotorSimple.Direction.FORWARD

    override fun setPower(power: Double) {
        this.power = power.coerceIn(-1.0, 1.0)
    }

    override fun getPower(): Double = power

    override fun getMotorType(): MotorConfigurationType =
        MotorConfigurationType.getUnspecifiedMotorType()

    override fun setMotorType(motorType: MotorConfigurationType?) {
        TODO("Not yet implemented")
    }

    override fun getController(): DcMotorController {
        TODO("Not yet implemented")
    }

    override fun getPortNumber(): Int = -1

    override fun setZeroPowerBehavior(zeroPowerBehavior: DcMotor.ZeroPowerBehavior?) {
        TODO("Not yet implemented")
    }

    override fun getZeroPowerBehavior(): DcMotor.ZeroPowerBehavior {
        TODO("Not yet implemented")
    }

    override fun setPowerFloat() {
        TODO("Not yet implemented")
    }

    override fun getPowerFloat(): Boolean {
        TODO("Not yet implemented")
    }

    override fun setTargetPosition(position: Int) {
        TODO("Not yet implemented")
    }

    override fun getTargetPosition(): Int {
        TODO("Not yet implemented")
    }

    override fun isBusy(): Boolean {
        TODO("Not yet implemented")
    }

    override fun getCurrentPosition(): Int = encoder

    override fun setMode(mode: DcMotor.RunMode?) {
        TODO("Not yet implemented")
    }

    override fun getMode(): DcMotor.RunMode {
        TODO("Not yet implemented")
    }

    override fun setMotorEnable() {
        TODO("Not yet implemented")
    }

    override fun setMotorDisable() {
        TODO("Not yet implemented")
    }

    override fun isMotorEnabled(): Boolean {
        TODO("Not yet implemented")
    }

    override fun setVelocity(angularRate: Double) {
        TODO("Not yet implemented")
    }

    override fun setVelocity(angularRate: Double, unit: AngleUnit?) {
        TODO("Not yet implemented")
    }

    override fun getVelocity(): Double = vel

    override fun getVelocity(unit: AngleUnit?): Double = when (unit) {
        AngleUnit.DEGREES -> vel / TPR * 360
        AngleUnit.RADIANS -> vel / TPR * 2 * PI
        null -> velocity
    }

    override fun setPIDCoefficients(mode: DcMotor.RunMode?, pidCoefficients: PIDCoefficients?) {
        TODO("Not yet implemented")
    }

    override fun setPIDFCoefficients(mode: DcMotor.RunMode?, pidfCoefficients: PIDFCoefficients?) {
        TODO("Not yet implemented")
    }

    override fun setVelocityPIDFCoefficients(p: Double, i: Double, d: Double, f: Double) {
        TODO("Not yet implemented")
    }

    override fun setPositionPIDFCoefficients(p: Double) {
        TODO("Not yet implemented")
    }

    override fun getPIDCoefficients(mode: DcMotor.RunMode?): PIDCoefficients {
        TODO("Not yet implemented")
    }

    override fun getPIDFCoefficients(mode: DcMotor.RunMode?): PIDFCoefficients {
        TODO("Not yet implemented")
    }

    override fun setTargetPositionTolerance(tolerance: Int) {
        TODO("Not yet implemented")
    }

    override fun getTargetPositionTolerance(): Int {
        TODO("Not yet implemented")
    }

    override fun getCurrent(unit: CurrentUnit?): Double {
        TODO("Not yet implemented")
    }

    override fun getCurrentAlert(unit: CurrentUnit?): Double {
        TODO("Not yet implemented")
    }

    override fun setCurrentAlert(current: Double, unit: CurrentUnit?) {
        TODO("Not yet implemented")
    }

    override fun isOverCurrent(): Boolean {
        TODO("Not yet implemented")
    }
}