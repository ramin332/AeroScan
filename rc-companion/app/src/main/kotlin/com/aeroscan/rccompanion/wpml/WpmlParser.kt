package com.aeroscan.rccompanion.wpml

import org.xmlpull.v1.XmlPullParser
import org.xmlpull.v1.XmlPullParserFactory
import java.io.ByteArrayInputStream
import java.io.InputStream
import java.util.zip.ZipInputStream

/**
 * Parses a Smart3D KMZ archive into an [ImportedKmz] (waypoints + ref point +
 * mission area polygon) and extracts the embedded `cloud.ply` bytes.
 *
 * Mirrors `flight_planner.kmz_import._parse_template` /
 * `_parse_waylines` / `_parse_smart_oblique_groups` field-for-field. The
 * resulting [ImportedKmz.toJsonString] round-trips through the Python
 * `intent_dict_to_imported_kmz` without translation.
 *
 * KMZ contents we touch:
 *   - `wpmz/template.kml`   (mission area polygon, takeoff ref point)
 *   - `wpmz/waylines.wpml`  (waypoints + smart oblique action groups)
 *   - `wpmz/res/ply/<name>/sfm_geo_desc.json` (preferred ENU origin; optional)
 *   - `wpmz/res/ply/<name>/cloud.ply` (the dense ICP target cloud)
 *
 * Everything else in the KMZ (3D-Tiles pyramid, mesh.bin) is ignored.
 */
object WpmlParser {

    private const val WPML_NS = "http://www.dji.com/wpmz/1.0.6"
    private const val KML_NS = "http://www.opengis.net/kml/2.2"

    /** Result holder: parsed waypoints + raw cloud.ply bytes (or null). */
    data class ParseResult(
        val intent: ImportedKmz,
        val cloudPlyBytes: ByteArray?,
    )

    fun parseKmz(kmzBytes: ByteArray, missionName: String): ParseResult {
        var templateBytes: ByteArray? = null
        var waylinesBytes: ByteArray? = null
        var sfmGeoDescBytes: ByteArray? = null
        var cloudPly: ByteArray? = null

        ZipInputStream(ByteArrayInputStream(kmzBytes)).use { zin ->
            while (true) {
                val entry = zin.nextEntry ?: break
                val name = entry.name.lowercase()
                val data = zin.readBytes()
                when {
                    name.endsWith("template.kml")       -> templateBytes = data
                    name.endsWith("waylines.wpml")      -> waylinesBytes = data
                    name.endsWith("sfm_geo_desc.json")  -> sfmGeoDescBytes = data
                    name.endsWith("cloud.ply")          -> cloudPly = data
                }
                zin.closeEntry()
            }
        }

        val waylines = waylinesBytes
            ?: throw IllegalArgumentException("KMZ does not contain wpmz/waylines.wpml")

        // Template parse (optional — derive polygon and takeoff ref).
        val (polygon, takeoffRef) = templateBytes
            ?.let { parseTemplate(it) }
            ?: (emptyList<DoubleArray>() to null)

        // sfm_geo_desc preferred for the ENU ref. Falls back to template
        // takeoff, then to the first waypoint.
        val ref: Triple<Double, Double, Double> = sfmGeoDescBytes?.let { parseSfmGeoDesc(it) }
            ?: takeoffRef
            ?: Triple(0.0, 0.0, 0.0)

        val waypoints = parseWaylines(waylines)

        // If we landed on (0,0,0) and have waypoints, anchor on the first.
        val effectiveRef = if (ref.first == 0.0 && ref.second == 0.0 && ref.third == 0.0
            && waypoints.isNotEmpty()) {
            Triple(waypoints[0].lat, waypoints[0].lon, waypoints[0].altEgm96)
        } else ref

        val intent = ImportedKmz(
            name = missionName,
            refLat = effectiveRef.first,
            refLon = effectiveRef.second,
            refAlt = effectiveRef.third,
            waypoints = waypoints,
            missionAreaWgs84 = polygon,
        )
        return ParseResult(intent, cloudPly)
    }

    // ---------------------------------------------------------------------
    // template.kml parse — polygon + takeoff ref
    // ---------------------------------------------------------------------

    /** Returns (polygon as [lon,lat,alt] triples, takeoff ref or null). */
    private fun parseTemplate(xml: ByteArray): Pair<List<DoubleArray>, Triple<Double, Double, Double>?> {
        val poly = mutableListOf<DoubleArray>()
        var takeoffLat: Double? = null
        var takeoffLon: Double? = null
        var takeoffAlt: Double? = null

        val parser = newParser(xml)
        var depth = 0
        var inPlacemark = false
        var inPoint = false

        while (true) {
            when (parser.next()) {
                XmlPullParser.START_TAG -> {
                    depth++
                    val name = parser.name ?: ""
                    val ns = parser.namespace ?: ""
                    if (ns == KML_NS) {
                        if (name == "Placemark") inPlacemark = true
                        if (inPlacemark && name == "Point") inPoint = true
                        if (inPlacemark && inPoint && name == "coordinates") {
                            val txt = parser.nextText()
                            for (line in txt.lines()) {
                                val parts = line.trim().split(",")
                                if (parts.size >= 2) {
                                    try {
                                        val lon = parts[0].toDouble()
                                        val lat = parts[1].toDouble()
                                        val alt = if (parts.size >= 3) parts[2].toDouble() else 0.0
                                        poly.add(doubleArrayOf(lon, lat, alt))
                                    } catch (e: NumberFormatException) {
                                        // skip
                                    }
                                }
                            }
                        }
                    }
                    if (ns == WPML_NS && name == "takeOffRefPoint") {
                        val txt = parser.nextText()
                        val parts = txt.trim().split(",")
                        if (parts.size >= 2) {
                            try {
                                takeoffLat = parts[0].toDouble()
                                takeoffLon = parts[1].toDouble()
                                takeoffAlt = if (parts.size >= 3) parts[2].toDouble() else 0.0
                            } catch (e: NumberFormatException) {
                                // skip
                            }
                        }
                    }
                }
                XmlPullParser.END_TAG -> {
                    val name = parser.name ?: ""
                    val ns = parser.namespace ?: ""
                    if (ns == KML_NS) {
                        if (name == "Placemark") inPlacemark = false
                        if (name == "Point") inPoint = false
                    }
                    depth--
                }
                XmlPullParser.END_DOCUMENT -> break
            }
        }

        val ref = if (takeoffLat != null && takeoffLon != null) {
            Triple(takeoffLat!!, takeoffLon!!, takeoffAlt ?: 0.0)
        } else null
        return poly to ref
    }

    // ---------------------------------------------------------------------
    // sfm_geo_desc.json parse — preferred ENU ref point
    // ---------------------------------------------------------------------

    private fun parseSfmGeoDesc(jsonBytes: ByteArray): Triple<Double, Double, Double>? {
        return try {
            val obj = org.json.JSONObject(String(jsonBytes, Charsets.UTF_8))
            val gps = obj.optJSONObject("ref_GPS") ?: return null
            val lat = gps.optDouble("latitude", 0.0)
            val lon = gps.optDouble("longitude", 0.0)
            val alt = gps.optDouble("altitude", 0.0)
            if (lat == 0.0 && lon == 0.0) null else Triple(lat, lon, alt)
        } catch (e: Exception) {
            null
        }
    }

    // ---------------------------------------------------------------------
    // waylines.wpml parse — waypoints + smart oblique groups
    // ---------------------------------------------------------------------

    private data class SmartObliqueGroup(
        val startIdx: Int,
        val endIdx: Int,
        val poses: List<SmartObliquePose>,
    )

    private fun parseWaylines(xml: ByteArray): List<ParsedWaypoint> {
        // Two passes: first collect smart oblique groups + default yaw base,
        // then collect placemarks. Single forward pass over an XML stream
        // would require carrying state across nesting; two passes is
        // simpler and within the size budget (~3 MB max).
        val groups = parseSmartObliqueGroups(xml)
        val defaultYawBase = parseDefaultYawBase(xml)
        return parsePlacemarks(xml, groups, defaultYawBase)
    }

    private fun parseDefaultYawBase(xml: ByteArray): String {
        val parser = newParser(xml)
        while (true) {
            when (parser.next()) {
                XmlPullParser.START_TAG -> {
                    if (parser.namespace == WPML_NS && parser.name == "gimbalHeadingYawBase") {
                        val txt = parser.nextText().trim()
                        if (txt.isNotEmpty()) return txt
                    }
                }
                XmlPullParser.END_DOCUMENT -> return "aircraft"
            }
        }
    }

    private fun parseSmartObliqueGroups(xml: ByteArray): List<SmartObliqueGroup> {
        val groups = mutableListOf<SmartObliqueGroup>()
        val parser = newParser(xml)
        // We're looking for actionGroup elements. Within each, collect:
        //   actionGroupStartIndex, actionGroupEndIndex, smartObliquePoint{
        //   smartObliqueEulerPitch, smartObliqueEulerYaw, smartObliqueEulerRoll }
        // and a marker actionActuatorFunc==startSmartOblique to qualify it.
        var inActionGroup = false
        var startIdx: Int? = null
        var endIdx: Int? = null
        var hasSmartOblique = false
        var poses: MutableList<SmartObliquePose> = mutableListOf()
        var inSmartObliquePoint = false
        var pitch = 0.0; var yaw = 0.0; var roll = 0.0
        var posePitchSet = false; var poseYawSet = false

        while (true) {
            when (parser.next()) {
                XmlPullParser.START_TAG -> {
                    val ns = parser.namespace ?: ""
                    val name = parser.name ?: ""
                    if (ns != WPML_NS) continue
                    when (name) {
                        "actionGroup" -> {
                            inActionGroup = true
                            startIdx = null; endIdx = null
                            hasSmartOblique = false
                            poses = mutableListOf()
                        }
                        "actionGroupStartIndex" -> if (inActionGroup) startIdx = parser.nextText().trim().toIntOrNull()
                        "actionGroupEndIndex"   -> if (inActionGroup) endIdx   = parser.nextText().trim().toIntOrNull()
                        "actionActuatorFunc" -> {
                            val txt = parser.nextText().trim()
                            if (inActionGroup && txt == "startSmartOblique") hasSmartOblique = true
                        }
                        "smartObliquePoint" -> {
                            inSmartObliquePoint = true
                            pitch = 0.0; yaw = 0.0; roll = 0.0
                            posePitchSet = false; poseYawSet = false
                        }
                        "smartObliqueEulerPitch" -> if (inSmartObliquePoint) {
                            pitch = parser.nextText().trim().toDoubleOrNull() ?: 0.0
                            posePitchSet = true
                        }
                        "smartObliqueEulerYaw" -> if (inSmartObliquePoint) {
                            yaw = parser.nextText().trim().toDoubleOrNull() ?: 0.0
                            poseYawSet = true
                        }
                        "smartObliqueEulerRoll" -> if (inSmartObliquePoint) {
                            roll = parser.nextText().trim().toDoubleOrNull() ?: 0.0
                        }
                    }
                }
                XmlPullParser.END_TAG -> {
                    val ns = parser.namespace ?: ""
                    val name = parser.name ?: ""
                    if (ns != WPML_NS) continue
                    when (name) {
                        "smartObliquePoint" -> {
                            if (inSmartObliquePoint && (posePitchSet || poseYawSet)) {
                                poses.add(SmartObliquePose(pitch, yaw, roll))
                            }
                            inSmartObliquePoint = false
                        }
                        "actionGroup" -> {
                            if (inActionGroup && hasSmartOblique
                                && startIdx != null && endIdx != null && poses.isNotEmpty()) {
                                groups.add(SmartObliqueGroup(startIdx!!, endIdx!!, poses.toList()))
                            }
                            inActionGroup = false
                        }
                    }
                }
                XmlPullParser.END_DOCUMENT -> return groups
            }
        }
    }

    private fun parsePlacemarks(
        xml: ByteArray,
        groups: List<SmartObliqueGroup>,
        defaultYawBase: String,
    ): List<ParsedWaypoint> {
        val out = mutableListOf<ParsedWaypoint>()
        val parser = newParser(xml)
        var inPlacemark = false
        // Per-placemark working values
        var lon = 0.0; var lat = 0.0
        var indexVal: Int? = null
        var executeHeight = 0.0
        var heading = 0.0
        var gimbalPitch = 0.0
        var gimbalYaw = 0.0
        var gimbalHeadingMode = "smoothTransition"
        var speed = 2.0

        while (true) {
            when (parser.next()) {
                XmlPullParser.START_TAG -> {
                    val ns = parser.namespace ?: ""
                    val name = parser.name ?: ""
                    if (ns == KML_NS) {
                        when (name) {
                            "Placemark" -> {
                                inPlacemark = true
                                lon = 0.0; lat = 0.0
                                indexVal = null
                                executeHeight = 0.0; heading = 0.0
                                gimbalPitch = 0.0; gimbalYaw = 0.0
                                gimbalHeadingMode = "smoothTransition"
                                speed = 2.0
                            }
                            "coordinates" -> if (inPlacemark) {
                                val txt = parser.nextText().trim()
                                val parts = txt.split(",")
                                if (parts.size >= 2) {
                                    lon = parts[0].toDoubleOrNull() ?: 0.0
                                    lat = parts[1].toDoubleOrNull() ?: 0.0
                                }
                            }
                        }
                    } else if (ns == WPML_NS && inPlacemark) {
                        when (name) {
                            "index" -> indexVal = parser.nextText().trim().toIntOrNull()
                            "executeHeight" -> executeHeight = parser.nextText().trim().toDoubleOrNull() ?: 0.0
                            "waypointHeadingAngle" -> heading = parser.nextText().trim().toDoubleOrNull() ?: 0.0
                            "waypointGimbalPitchAngle" -> gimbalPitch = parser.nextText().trim().toDoubleOrNull() ?: 0.0
                            "waypointGimbalYawAngle" -> gimbalYaw = parser.nextText().trim().toDoubleOrNull() ?: 0.0
                            "waypointGimbalHeadingMode" -> {
                                val txt = parser.nextText().trim()
                                if (txt.isNotEmpty()) gimbalHeadingMode = txt
                            }
                            "waypointSpeed" -> speed = parser.nextText().trim().toDoubleOrNull() ?: 2.0
                        }
                    }
                }
                XmlPullParser.END_TAG -> {
                    if (parser.namespace == KML_NS && parser.name == "Placemark" && inPlacemark) {
                        val idx = indexVal ?: out.size
                        val poses = groups.firstOrNull { idx in it.startIdx..it.endIdx }?.poses ?: emptyList()
                        out.add(ParsedWaypoint(
                            index = idx,
                            lon = lon, lat = lat,
                            altEgm96 = executeHeight,
                            headingDeg = heading,
                            gimbalPitchDeg = gimbalPitch,
                            speedMs = speed,
                            gimbalYawRawDeg = gimbalYaw,
                            gimbalHeadingMode = gimbalHeadingMode,
                            gimbalYawBase = defaultYawBase,
                            smartObliquePoses = poses,
                        ))
                        inPlacemark = false
                    }
                }
                XmlPullParser.END_DOCUMENT -> return out
            }
        }
    }

    // ---------------------------------------------------------------------
    // shared
    // ---------------------------------------------------------------------

    private fun newFactory(): XmlPullParserFactory {
        // On Android, the platform provides a working default
        // XmlPullParserFactory. On JVM unit tests, android.jar's
        // XmlPullParserFactory is a stub (throws "not mocked"); we add
        // kxml2 to the test classpath and request it by name here.
        // newInstance() with no arguments first tries the system property
        // org.xmlpull.v1.XmlPullParserFactory; on Android that's set to
        // the platform impl, on JVM tests we fall back to kxml2 explicitly.
        val factory: XmlPullParserFactory = try {
            XmlPullParserFactory.newInstance()
        } catch (e: Throwable) {
            // JVM test path — kxml2 in test classpath
            XmlPullParserFactory.newInstance(
                "org.kxml2.io.KXmlParser,org.kxml2.io.KXmlSerializer", null,
            )
        }
        factory.isNamespaceAware = true
        return factory
    }

    /** Strip a leading UTF-8/UTF-16 BOM and any whitespace bytes before the
     * first '<'. Some upstream tools (and our own multiline test fixtures
     * with `.trimIndent()`) emit XML with a leading newline, which kxml2's
     * strict mode rejects with `"PI must not start with xml"`. */
    private fun stripXmlPreamble(xml: ByteArray): ByteArray {
        var start = 0
        // UTF-8 BOM (EF BB BF)
        if (xml.size >= 3 && xml[0] == 0xEF.toByte() && xml[1] == 0xBB.toByte() && xml[2] == 0xBF.toByte()) {
            start = 3
        }
        while (start < xml.size) {
            val b = xml[start].toInt()
            if (b == 0x20 || b == 0x09 || b == 0x0A || b == 0x0D) start++ else break
        }
        return if (start == 0) xml else xml.copyOfRange(start, xml.size)
    }

    private fun newParser(xml: ByteArray): XmlPullParser =
        newFactory().newPullParser().apply {
            setInput(ByteArrayInputStream(stripXmlPreamble(xml)), "UTF-8")
        }

    @Suppress("unused") // symmetry with the byte-array variant
    private fun newParser(stream: InputStream): XmlPullParser =
        newFactory().newPullParser().apply { setInput(stream, "UTF-8") }
}
