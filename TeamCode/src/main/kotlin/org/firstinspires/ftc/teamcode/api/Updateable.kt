package org.firstinspires.ftc.teamcode.api

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import kotlin.math.abs

/**
 * Standardizes components in a way that they can be manipulated by classes like HardwareComposition
 *
 * @see HardwareComposition
 */
interface Updateable {
    val busy: Boolean

    fun update()
}

/**
 * An Updateable that composes other Updateables to let them work together
 *
 */
open class HardwareComposition(val opMode: LinearOpMode) : Updateable {
    val components = mutableListOf<Updateable>()

    override val busy: Boolean
        get() = (components.sumBy { if (it.busy) 1 else 0 } > 0)

    /**
     * Registers an updatable with `this`
     */
    protected fun <T : Updateable> T.register(): T {
        components.add(this)
        return this
    }

    override fun update() = components.forEach { it.update() }

    /**
     * Waits for busy to become false by running update
     */
    fun waitForIdle() {
        while (!opMode.isStopRequested && !Thread.currentThread().isInterrupted && busy) update()
    }
}

/**
 * A wrapper around Servo that is updateable
 */
@Config
class UpdateableServo(s: Servo) : Updateable, Servo by s {
    companion object {
        @JvmField
        var MOVE_BY = 0.25
    }

    var targetPosition: Double? = null

    override val busy: Boolean
        get() = when (targetPosition) {
            null -> false
            else -> abs(position - targetPosition!!) > MOVE_BY
        }

    override fun update() {
        if (targetPosition != null) {
            if (position > targetPosition!!)
                position -= MOVE_BY
            else if (position < targetPosition!!)
                position += MOVE_BY
        }
    }
}

class UpdateableMotor(m: DcMotorEx): Updateable, DcMotorEx by m {
    init {
        this.mode = DcMotor.RunMode.RUN_TO_POSITION
    }

    override val busy: Boolean
        get() = this.isBusy

    override fun update() {
        // There's nothing to be done here
    }

}