package org.firstinspires.ftc.teamcode.api

import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.api.ToggleableGamepad.Mode.NORMAL
import org.firstinspires.ftc.teamcode.api.ToggleableGamepad.Mode.TOGGLE
import org.firstinspires.ftc.teamcode.api.ToggleableGamepad.Mode.DEBOUNCE
import org.firstinspires.ftc.teamcode.api.framework.Updatable
import org.firstinspires.ftc.teamcode.api.framework.UpdatableComposition
import org.firstinspires.ftc.teamcode.api.framework.noop

class ToggleableGamepad(
        val gamepad: Gamepad,

        aMode: Mode = NORMAL,
        bMode: Mode = NORMAL,
        xMode: Mode = NORMAL,
        yMode: Mode = NORMAL,

        rbMode: Mode = NORMAL,
        lbMode: Mode = NORMAL,

        dpadMode: Mode = NORMAL
): UpdatableComposition() {
    init { update() }

    val a = initBtn(aMode) { a }
    val b = initBtn(bMode) { b }
    val x = initBtn(xMode) { x }
    val y = initBtn(yMode) { y }

    val rightBumper = initBtn(rbMode) { right_bumper }
    val leftBumper = initBtn(lbMode) { left_bumper }

    val dpadUp = initBtn(dpadMode) { dpad_up }
    val dpadDown = initBtn(dpadMode) { dpad_down }
    val dpadLeft = initBtn(dpadMode) { dpad_left }
    val dpadRight = initBtn(dpadMode) { dpad_right }

    val leftStickX: Double
        get() = gamepad.left_stick_x.toDouble()
    val leftStickY: Double
        get() = -gamepad.left_stick_y.toDouble()

    val rightStickX: Double
        get() = gamepad.right_stick_x.toDouble()
    val rightStickY: Double
        get() = -gamepad.right_stick_y.toDouble()

    val leftTrigger: Double
        get() = gamepad.left_trigger.toDouble()
    val rightTrigger: Double
        get() = gamepad.right_trigger.toDouble()

    // The actual implementation

    enum class Mode { NORMAL, TOGGLE, DEBOUNCE }

    interface Queryable : Updatable {
        override val busy: Boolean
            get() = false

        var value: Boolean
    }

    private inner class Passthrough(val query: Gamepad.() -> Boolean) : Queryable {
        override var value: Boolean
            get() = gamepad.query()
            set(value) = noop

        override fun update() = noop
    }

    private inner class Toggle(val query: Gamepad.() -> Boolean) : Queryable {
        private var wasDown = false
        override var value = false

        override fun update() {
            val down = gamepad.query()
            if (down && !wasDown)
                value = !value
            wasDown = down
        }
    }

    private inner class Debounce(val query: Gamepad.() -> Boolean) : Queryable {
        private var wasDown = false
        override var value: Boolean = false

        override fun update() {
            val down = gamepad.query()
            if (down && !wasDown)
                value = true
            else if (down && wasDown)
                value = false
            wasDown = down
        }
    }

    private fun initBtn(mode: Mode, query: Gamepad.() -> Boolean) = when (mode) {
        NORMAL -> Passthrough(query).register()
        TOGGLE -> Toggle(query).register()
        DEBOUNCE -> Debounce(query).register()
    }

}