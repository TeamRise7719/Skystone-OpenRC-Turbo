package com.qualcomm.robotcore.eventloop.opmode;

import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
/**
 * @Author Sean Cardosi
 * @Date 11/7/19
 */

/**
 * Provides a way to temporarily disable an OpMode annotated with
 * {@link Autonomous} or {@link TeleOp} from showing up
 * on the driver station OpMode list.
 *
 * @see Autonomous
 * @see TeleOp
 */
@Documented
@Target(ElementType.TYPE)
@Retention(RetentionPolicy.RUNTIME)
public @interface Archived
{
}
