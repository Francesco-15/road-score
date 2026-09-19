package it.apc.appmappa

import android.Manifest
import android.annotation.SuppressLint
import android.app.Activity
import android.content.Intent
import android.content.pm.PackageManager
import android.graphics.Bitmap
import android.graphics.Canvas
import android.location.Location
import android.location.LocationListener
import android.location.LocationManager
import android.net.Uri
import android.os.Build
import android.os.Bundle
import android.webkit.WebChromeClient
import android.webkit.WebSettings
import android.webkit.WebView
import android.webkit.WebViewClient
import android.widget.Button
import android.widget.TextView
import androidx.core.content.FileProvider
import java.io.File
import java.util.Locale

class MainActivity : Activity() {
    private lateinit var session: RoadSession
    private lateinit var mapView: WebView
    private lateinit var statusText: TextView
    private lateinit var metricsText: TextView
    private lateinit var sessionButton: Button
    private lateinit var bleClient: BleRoadClient
    private lateinit var locationManager: LocationManager
    private var recording = false
    private var mapReady = false
    private var lastAccuracy: Float? = null
    private var firmwareScore: String? = null
    private var detectionThreshold: Double? = null
    private val locationListener = LocationListener { location ->
        if (!recording) return@LocationListener
        lastAccuracy = location.accuracy
        session.addLocation(location)
        refreshUi()
    }
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState); setContentView(R.layout.activity_main)
        session = RoadSession(); mapView = findViewById(R.id.mapView); statusText = findViewById(R.id.statusText)
        metricsText = findViewById(R.id.metricsText); sessionButton = findViewById(R.id.sessionButton)
        locationManager = getSystemService(LOCATION_SERVICE) as LocationManager
        bleClient = BleRoadClient(this, ::setStatus, ::handleBleLine)
        configureMap()
        findViewById<Button>(R.id.connectButton).setOnClickListener {
            if (!hasBluetoothPermission()) { requestRuntimePermissions(); setStatus("Concedi i permessi Bluetooth, poi riprova"); return@setOnClickListener }
            bleClient.connect()
        }
        sessionButton.setOnClickListener { if (recording) stopSession() else startSession() }
        findViewById<Button>(R.id.exportButton).setOnClickListener { shareCsv() }
        findViewById<Button>(R.id.saveMapButton).setOnClickListener { saveMapImage() }
        findViewById<Button>(R.id.followGpsButton).setOnClickListener {
            mapView.evaluateJavascript("window.followGps && window.followGps();", null)
            setStatus("Seguimento GPS attivo")
        }
        requestRuntimePermissions(); refreshUi()
    }
    @SuppressLint("SetJavaScriptEnabled") private fun configureMap() {
        mapView.settings.javaScriptEnabled = true; mapView.settings.domStorageEnabled = true
        // Non archiviare sul telefono le tessere delle zone non necessarie:
        // il dato persistente della sessione rimane soltanto percorso/eventi esportati.
        mapView.settings.cacheMode = WebSettings.LOAD_NO_CACHE
        mapView.webViewClient = object : WebViewClient() { override fun onPageFinished(view: WebView, url: String) { mapReady = true; refreshUi() } }
        mapView.webChromeClient = WebChromeClient(); mapView.loadUrl("file:///android_asset/map.html")
    }
    private fun requestRuntimePermissions() {
        val wanted = mutableListOf(Manifest.permission.ACCESS_FINE_LOCATION)
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) wanted += listOf(Manifest.permission.BLUETOOTH_SCAN, Manifest.permission.BLUETOOTH_CONNECT)
        val missing = wanted.filter { checkSelfPermission(it) != PackageManager.PERMISSION_GRANTED }
        if (missing.isNotEmpty()) requestPermissions(missing.toTypedArray(), 100)
    }
    private fun hasBluetoothPermission() = Build.VERSION.SDK_INT < Build.VERSION_CODES.S || checkSelfPermission(Manifest.permission.BLUETOOTH_SCAN) == PackageManager.PERMISSION_GRANTED
    @SuppressLint("MissingPermission") private fun startSession() {
        if (checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED) { requestRuntimePermissions(); setStatus("Consenti la posizione precisa per avviare"); return }
        session.start(); session.updateThreshold(detectionThreshold); mapView.clearCache(true); recording = true; sessionButton.text = "Termina"; setStatus("Registrazione GPS attiva – attendi un fix preciso")
        try { locationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 1000L, 1f, locationListener); locationManager.requestLocationUpdates(LocationManager.NETWORK_PROVIDER, 2000L, 2f, locationListener) }
        catch (_: IllegalArgumentException) { setStatus("Attiva Posizione e GPS nelle impostazioni") }; refreshUi()
    }
    private fun stopSession() { recording = false; locationManager.removeUpdates(locationListener); sessionButton.text = "Avvia"; setStatus("Sessione terminata"); refreshUi() }
    private fun handleBleLine(line: String) = runOnUiThread {
        when { line.startsWith("\$EVENT") -> parseEvent(line)?.let { event -> session.addEvent(event.first, event.second, event.third, detectionThreshold); setStatus("Evento ${event.first}: severità ${event.second.toInt()}/100"); refreshUi() }
            line.startsWith("\$STATUS") -> { firmwareScore = statusValue(line, "SCORE"); detectionThreshold = statusValue(line, "TH")?.toDoubleOrNull(); session.updateThreshold(detectionThreshold); refreshUi() } }
    }
    private fun parseEvent(line: String): Triple<String, Double, Double>? {
        val fields = line.split(','); if (fields.size < 3) return null
        fun value(name: String) = fields.firstOrNull { it.startsWith("$name:", true) }?.substringAfter(':')?.toDoubleOrNull() ?: 0.0
        return Triple(fields[2].uppercase(Locale.ROOT), value("SEV"), value("CONF"))
    }
    private fun statusValue(line: String, name: String): String? = line.split(',')
        .firstOrNull { it.startsWith("$name:", true) }?.substringAfter(':')
    private fun refreshUi() = runOnUiThread {
        val accuracy = lastAccuracy?.let { " · GPS ±${it.toInt()} m" } ?: " · GPS in attesa"
        metricsText.text = "Distanza: ${session.distance.toInt()} m · Eventi: ${session.events.size} · Score firmware: ${firmwareScore ?: "—"} · TH: ${detectionThreshold?.toInt() ?: "—"}$accuracy"; updateMap()
    }
    private fun updateMap() {
        if (!mapReady) return
        val points = session.points.joinToString(",") { "[${it.latitude},${it.longitude},${it.segment}]" }
        val events = session.events.joinToString(",") { e -> if (e.latitude == null) "null" else "[${e.latitude},${e.longitude},${e.severity},${e.confidence},'${e.type.replace("'", "")}']" }
        val scores = (0..session.currentSegment).joinToString(",") { "[${session.scoreFor(it)},${session.eventCountFor(it)}]" }
        mapView.evaluateJavascript("window.updateRoad([$points],[$events],[$scores]);", null)
    }
    private fun setStatus(message: String) = runOnUiThread { statusText.text = message }
    private fun shareCsv() {
        val dir = File(cacheDir, "exports").apply { mkdirs() }; val file = File(dir, "mappa_stradale_${System.currentTimeMillis()}.csv")
        file.printWriter().use { out -> out.println("timestamp_ms,event_class,severity,confidence,threshold,latitude,longitude,gps_accuracy_m,segment"); session.events.forEach { e -> out.println("${e.elapsedMs},${e.type},${e.severity},${e.confidence},${e.threshold ?: ""},${e.latitude ?: ""},${e.longitude ?: ""},${e.accuracyMetres ?: ""},${e.segment ?: ""}") } }
        val uri: Uri = FileProvider.getUriForFile(this, "$packageName.files", file)
        startActivity(Intent.createChooser(Intent(Intent.ACTION_SEND).apply { type = "text/csv"; putExtra(Intent.EXTRA_STREAM, uri); addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION) }, "Esporta sessione mappa"))
    }
    private fun saveMapImage() {
        if (mapView.width == 0 || mapView.height == 0) { setStatus("Mappa non ancora pronta da salvare"); return }
        val bitmap = Bitmap.createBitmap(mapView.width, mapView.height, Bitmap.Config.ARGB_8888)
        mapView.draw(Canvas(bitmap))
        val dir = File(cacheDir, "exports").apply { mkdirs() }
        val file = File(dir, "mappa_stradale_${System.currentTimeMillis()}.png")
        file.outputStream().use { bitmap.compress(Bitmap.CompressFormat.PNG, 100, it) }
        bitmap.recycle()
        val uri = FileProvider.getUriForFile(this, "$packageName.files", file)
        setStatus("Immagine della mappa pronta per il salvataggio")
        startActivity(Intent.createChooser(Intent(Intent.ACTION_SEND).apply { type = "image/png"; putExtra(Intent.EXTRA_STREAM, uri); addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION) }, "Salva o condividi la mappa"))
    }
    override fun onDestroy() { if (checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED) locationManager.removeUpdates(locationListener); bleClient.close(); mapView.destroy(); super.onDestroy() }
}
