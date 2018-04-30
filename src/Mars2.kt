import java.util.Scanner

/**
 * Auto-generated code below aims at helping you parse
 * the standard input according to the problem statement.
 **/
fun main(args: Array<String>) {
    val input = Scanner(System.`in`)
    val surfaceN = input.nextInt() // the number of points used to draw the surface of Mars.
    val points = (0 until surfaceN).map { Point(input.nextInt(), input.nextInt()) }
    val surface = Surface(points)

    // game loop
    while (true) {
        val x = input.nextInt()
        val y = input.nextInt()
        val hSpeed = input.nextInt() // the horizontal speed (in m/s), can be negative.
        val vSpeed = input.nextInt() // the vertical speed (in m/s), can be negative.
        val fuel = input.nextInt() // the quantity of remaining fuel in liters.
        val rotate = input.nextInt() // the rotation angle in degrees (-90 to 90).
        val power = input.nextInt() // the thrust power (0 to 4).

        val lander = Lander(Point(x, y), Speed(hSpeed, vSpeed), fuel, rotate, power)

        if (!surface.isOverLandingSegment(lander)) {
            if (surface.goesInWrongDirection(lander) || surface.tooFastHorizontally(lander))
                println("${surface.angleToSlow(lander)} 4")
            else if (surface.tooSlowHorizontally(lander))
                println("${surface.angleToAimTarget(lander)} 4")
            else println("0 ${surface.powerToHover(lander)}")
        } else
            if (surface.isFinishing(lander))
                println("0 3")
            else if (surface.hasSafeSpeed(lander))
                println("0 2")
            else println("${surface.angleToSlow(lander)} 4")
    }
}

fun debug(msg: Any) = System.err.println("$msg")

class Point(val x: Int, val y: Int) {
    override fun toString() = "($x, $y)"
}

class Speed(val x: Int, val y: Int) {
    fun mag() = Math.sqrt((x * x + y * y).toDouble())
}

class Lander(val position: Point, val speed: Speed, val fuel: Int, val rotate: Int, val power: Int)

const val Y_MARGIN = 20
const val SPEED_MARGIN = 5
const val MAX_DY = 40
const val MAX_DX = 20
const val GRAVITY = 3.711

class Surface(points: List<Point>) {
    private val landingSegment: Pair<Point, Point>
    private val segments: List<Pair<Point, Point>>
    private val angleConst: Int

    init {
        segments = points.dropLast(1).mapIndexed { idx, point -> Pair(point, points[idx + 1]) }
        debug(segments)
        landingSegment = segments.first { it.first.y == it.second.y }
        debug(landingSegment)
        angleConst = Math.toDegrees(Math.acos(GRAVITY / 4.0)).toInt()
    }

    fun isOverLandingSegment(lander: Lander) = (lander.position.x >= landingSegment.first.x && lander.position.x <= landingSegment.second.x)

    fun isFinishing(lander: Lander) = lander.position.y < (landingSegment.first.y + Y_MARGIN)

    fun goesInWrongDirection(lander: Lander) = (lander.position.x < landingSegment.first.x && lander.speed.x < 0) || (lander.position.x > landingSegment.second.x && lander.speed.x > 0)

    fun tooFastHorizontally(lander: Lander) = Math.abs(lander.speed.x) > 4 * MAX_DX

    fun tooSlowHorizontally(lander: Lander) = Math.abs(lander.speed.x) < 2 * MAX_DX

    fun hasSafeSpeed(lander: Lander) = Math.abs(lander.speed.x) <= MAX_DX - SPEED_MARGIN && Math.abs(lander.speed.y) <= MAX_DY - SPEED_MARGIN

    fun angleToSlow(lander: Lander) = Math.toDegrees(Math.asin(lander.speed.x / lander.speed.mag())).toInt()

    fun angleToAimTarget(lander: Lander) = if (lander.position.x < landingSegment.first.x) -angleConst else if (lander.position.x > landingSegment.second.x) angleConst else 0

    fun powerToHover(lander: Lander) = if (lander.speed.y >= 0) 3 else 4
}