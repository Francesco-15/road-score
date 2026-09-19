package it.apc.appmappa

import android.location.Location
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max

data class RoadPoint(val latitude: Double, val longitude: Double, val routeMetres: Double, val segment: Int)
data class RoadEvent(val elapsedMs: Long, val type: String, val severity: Double, val confidence: Double, val threshold: Double?, val latitude: Double?, val longitude: Double?, val accuracyMetres: Float?, val segment: Int?)

/** Sessione locale: combina baseline del manto ed eventi su micro-tratti da 10 m. */
class RoadSession {
    companion object {
        // 10 m è abbastanza maggiore dell'errore GPS tipico (~3 m), ma localizza una buca.
        const val SEGMENT_METRES = 10.0
        private const val GPS_NOISE_FLOOR_METRES = 3.0
        private const val MAX_ACCURACY_ACCEPTED_METRES = 15f
        private const val GPS_STALE_MS = 15_000L
        private const val MAX_JUMP_METRES = 200.0
    }
    private var lastRawLocation: Location? = null
    private var lastPoint: RoadPoint? = null
    private var lastLocationAt = 0L
    private var distanceMetres = 0.0
    private var currentThreshold: Double? = null
    private val baselineScores = mutableMapOf<Int, Double>()
    private val penalties = mutableMapOf<Int, Double>()
    private val eventCounts = mutableMapOf<Int, Int>()
    private val roadPoints = mutableListOf<RoadPoint>()
    val events = mutableListOf<RoadEvent>()
    val points: List<RoadPoint> get() = roadPoints
    val distance: Double get() = distanceMetres
    val currentSegment: Int get() = (distanceMetres / SEGMENT_METRES).toInt()

    fun start() { lastRawLocation = null; lastPoint = null; lastLocationAt = 0L; distanceMetres = 0.0; currentThreshold = null; baselineScores.clear(); penalties.clear(); eventCounts.clear(); roadPoints.clear(); events.clear() }

    /** Aggiorna la baseline usata dai tratti percorsi dopo questo stato firmware. */
    fun updateThreshold(threshold: Double?) { currentThreshold = threshold?.takeIf { it > 0.0 } }

    /** Ignora fix imprecisi e micro-spostamenti compatibili con un errore GPS di circa ±3 m. */
    fun addLocation(location: Location): Boolean {
        if (!location.hasAccuracy() || location.accuracy > MAX_ACCURACY_ACCEPTED_METRES) return false
        val oldRaw = lastRawLocation
        if (oldRaw != null && oldRaw.distanceTo(location) > MAX_JUMP_METRES) return false
        lastRawLocation = Location(location); lastLocationAt = System.currentTimeMillis()
        val previous = lastPoint
        if (previous == null) {
            baselineScores[0] = baselineScore(currentThreshold)
            roadPoints += RoadPoint(location.latitude, location.longitude, 0.0, 0); lastPoint = roadPoints.last(); return true
        }
        val dx = (location.longitude - previous.longitude) * 111_320.0 * cos(Math.toRadians(previous.latitude))
        val dy = (location.latitude - previous.latitude) * 111_132.0
        if (hypot(dx, dy) < max(GPS_NOISE_FLOOR_METRES, location.accuracy.toDouble())) return false
        // Filtro leggero: riduce il rumore senza appiattire curve e svolte.
        val lat = previous.latitude + (location.latitude - previous.latitude) * 0.55
        val lon = previous.longitude + (location.longitude - previous.longitude) * 0.55
        val step = Location("").apply { latitude = previous.latitude; longitude = previous.longitude }
            .distanceTo(Location("").apply { latitude = lat; longitude = lon }).toDouble()
        if (step < GPS_NOISE_FLOOR_METRES) return false
        distanceMetres += step
        val segment = currentSegment
        baselineScores.putIfAbsent(segment, baselineScore(currentThreshold))
        roadPoints += RoadPoint(lat, lon, distanceMetres, segment)
        lastPoint = roadPoints.last()
        return true
    }

    fun addEvent(type: String, severity: Double, confidence: Double, threshold: Double?) {
        val sev = severity.coerceIn(0.0, 100.0); val conf = confidence.coerceIn(0.0, 100.0)
        val point = lastPoint; val fresh = point != null && System.currentTimeMillis() - lastLocationAt <= GPS_STALE_MS
        val segment = if (fresh) currentSegment else null
        events += RoadEvent(System.currentTimeMillis(), type, sev, conf, threshold, if (fresh) point!!.latitude else null, if (fresh) point!!.longitude else null, lastRawLocation?.accuracy, segment)
        if (segment != null) {
            baselineScores.putIfAbsent(segment, baselineScore(threshold ?: currentThreshold))
            // La gravità incide più del semplice colore del singolo evento;
            // la somma rende visibile anche la concentrazione nel tratto di 50 m.
            val severityPenalty = when {
                sev >= 80.0 -> 45.0 // buca/problema critico
                sev >= 60.0 -> 28.0 // dissesto importante
                sev >= 40.0 -> 20.0 // anomalia gialla: deve rendere visibile una singola buca
                else -> 8.0         // lieve irregolarità
            }
            val confidenceWeight = 0.6 + conf / 250.0 // da 0,6 a 1,0
            // TH basso: baseline buona, quindi il singolo evento è molto impattante.
            // TH alto: baseline già degradata, evento con peso aggiuntivo minore.
            val thresholdWeight = threshold?.takeIf { it > 0.0 }
                ?.let { (128.0 / it).coerceIn(0.65, 1.60) } ?: 1.0
            val count = (eventCounts[segment] ?: 0) + 1
            val densityWeight = 1.0 + ((count - 1) * 0.12).coerceAtMost(0.72)
            penalties[segment] = (penalties[segment] ?: 0.0) + severityPenalty * confidenceWeight * thresholdWeight * densityWeight
            eventCounts[segment] = count
        }
    }
    fun scoreFor(segment: Int): Double = ((baselineScores[segment] ?: baselineScore(currentThreshold)) - (penalties[segment] ?: 0.0)).coerceAtLeast(0.0)
    fun eventCountFor(segment: Int): Int = eventCounts[segment] ?: 0

    /** TH=103 osservato da fermo resta nella baseline verde; TH alto degrada il manto. */
    private fun baselineScore(threshold: Double?): Double {
        val th = threshold ?: return 100.0
        return when {
            th <= 110.0 -> 100.0
            th <= 160.0 -> 100.0 - (th - 110.0) * 20.0 / 50.0 // fino a giallo
            th <= 210.0 -> 80.0 - (th - 160.0) * 25.0 / 50.0  // fino ad arancione
            else -> (55.0 - (th - 210.0) * 15.0 / 45.0).coerceAtLeast(40.0)
        }
    }
}
