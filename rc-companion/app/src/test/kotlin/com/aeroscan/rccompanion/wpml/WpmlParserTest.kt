package com.aeroscan.rccompanion.wpml

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Test
import java.io.ByteArrayOutputStream
import java.util.zip.ZipEntry
import java.util.zip.ZipOutputStream

class WpmlParserTest {

    @Test
    fun `parses minimal KMZ with one waypoint`() {
        val template = """
            <?xml version="1.0" encoding="UTF-8"?>
            <kml xmlns="http://www.opengis.net/kml/2.2"
                 xmlns:wpml="http://www.dji.com/wpmz/1.0.6">
              <Document>
                <wpml:missionConfig>
                  <wpml:takeOffRefPoint>53.123456,5.654321,12.5</wpml:takeOffRefPoint>
                </wpml:missionConfig>
                <Placemark>
                  <Point>
                    <coordinates>5.6540,53.1233,0.0
5.6543,53.1233,0.0
5.6543,53.1236,0.0
5.6540,53.1236,0.0
</coordinates>
                  </Point>
                </Placemark>
              </Document>
            </kml>
        """.trimIndent().trim()

        val waylines = """
            <?xml version="1.0" encoding="UTF-8"?>
            <kml xmlns="http://www.opengis.net/kml/2.2"
                 xmlns:wpml="http://www.dji.com/wpmz/1.0.6">
              <Document>
                <wpml:missionConfig>
                  <wpml:gimbalHeadingYawBase>aircraft</wpml:gimbalHeadingYawBase>
                </wpml:missionConfig>
                <Folder>
                  <Placemark>
                    <Point>
                      <coordinates>5.6541,53.1234,0.0</coordinates>
                    </Point>
                    <wpml:index>0</wpml:index>
                    <wpml:executeHeight>15.0</wpml:executeHeight>
                    <wpml:waypointSpeed>2.5</wpml:waypointSpeed>
                    <wpml:waypointHeadingParam>
                      <wpml:waypointHeadingAngle>42.0</wpml:waypointHeadingAngle>
                    </wpml:waypointHeadingParam>
                    <wpml:waypointGimbalHeadingParam>
                      <wpml:waypointGimbalPitchAngle>-15.0</wpml:waypointGimbalPitchAngle>
                      <wpml:waypointGimbalYawAngle>10.0</wpml:waypointGimbalYawAngle>
                      <wpml:waypointGimbalHeadingMode>smoothTransition</wpml:waypointGimbalHeadingMode>
                    </wpml:waypointGimbalHeadingParam>
                  </Placemark>
                </Folder>
              </Document>
            </kml>
        """.trimIndent().trim()

        val kmzBytes = buildKmz(mapOf(
            "wpmz/template.kml" to template.toByteArray(),
            "wpmz/waylines.wpml" to waylines.toByteArray(),
        ))

        val result = WpmlParser.parseKmz(kmzBytes, "TestSite")
        assertEquals("TestSite", result.intent.name)
        assertEquals(1, result.intent.waypoints.size)
        val wp = result.intent.waypoints[0]
        assertEquals(0, wp.index)
        assertEquals(5.6541, wp.lon, 1e-9)
        assertEquals(53.1234, wp.lat, 1e-9)
        assertEquals(15.0, wp.altEgm96, 1e-9)
        assertEquals(42.0, wp.headingDeg, 1e-9)
        assertEquals(-15.0, wp.gimbalPitchDeg, 1e-9)
        assertEquals(10.0, wp.gimbalYawRawDeg, 1e-9)
        assertEquals("aircraft", wp.gimbalYawBase)
        assertEquals(2.5, wp.speedMs, 1e-9)
        // takeoff ref overrides default 0/0/0
        assertEquals(53.123456, result.intent.refLat, 1e-9)
        assertEquals(5.654321, result.intent.refLon, 1e-9)
        assertEquals(12.5, result.intent.refAlt, 1e-9)
        // polygon ingested
        assertEquals(4, result.intent.missionAreaWgs84.size)
        assertEquals(5.6540, result.intent.missionAreaWgs84[0][0], 1e-9)
    }

    @Test
    fun `parses smart oblique action group attached to waypoint`() {
        val waylines = """
            <?xml version="1.0" encoding="UTF-8"?>
            <kml xmlns="http://www.opengis.net/kml/2.2"
                 xmlns:wpml="http://www.dji.com/wpmz/1.0.6">
              <Document>
                <Folder>
                  <Placemark>
                    <Point><coordinates>5.0,53.0,0.0</coordinates></Point>
                    <wpml:index>0</wpml:index>
                    <wpml:executeHeight>10.0</wpml:executeHeight>
                  </Placemark>
                  <wpml:actionGroup>
                    <wpml:actionGroupId>1</wpml:actionGroupId>
                    <wpml:actionGroupStartIndex>0</wpml:actionGroupStartIndex>
                    <wpml:actionGroupEndIndex>0</wpml:actionGroupEndIndex>
                    <wpml:action>
                      <wpml:actionActuatorFunc>startSmartOblique</wpml:actionActuatorFunc>
                      <wpml:actionActuatorFuncParam>
                        <wpml:smartObliquePoint>
                          <wpml:smartObliqueEulerPitch>-19.0</wpml:smartObliqueEulerPitch>
                          <wpml:smartObliqueEulerYaw>0.0</wpml:smartObliqueEulerYaw>
                          <wpml:smartObliqueEulerRoll>0.0</wpml:smartObliqueEulerRoll>
                        </wpml:smartObliquePoint>
                        <wpml:smartObliquePoint>
                          <wpml:smartObliqueEulerPitch>-30.0</wpml:smartObliqueEulerPitch>
                          <wpml:smartObliqueEulerYaw>30.0</wpml:smartObliqueEulerYaw>
                          <wpml:smartObliqueEulerRoll>0.0</wpml:smartObliqueEulerRoll>
                        </wpml:smartObliquePoint>
                      </wpml:actionActuatorFuncParam>
                    </wpml:action>
                  </wpml:actionGroup>
                </Folder>
              </Document>
            </kml>
        """.trimIndent().trim()
        val kmzBytes = buildKmz(mapOf("wpmz/waylines.wpml" to waylines.toByteArray()))
        val result = WpmlParser.parseKmz(kmzBytes, "X")
        val wp = result.intent.waypoints[0]
        assertEquals(2, wp.smartObliquePoses.size)
        assertEquals(-19.0, wp.smartObliquePoses[0].pitchDeg, 1e-9)
        assertEquals(30.0, wp.smartObliquePoses[1].yawOffsetDeg, 1e-9)
    }

    @Test
    fun `JSON round trip matches Python schema field names`() {
        val intent = ImportedKmz(
            name = "RoundTrip",
            refLat = 53.0, refLon = 5.0, refAlt = 10.0,
            waypoints = listOf(
                ParsedWaypoint(
                    index = 0, lon = 5.001, lat = 53.001, altEgm96 = 15.0,
                    headingDeg = 42.0, gimbalPitchDeg = -15.0, speedMs = 2.5,
                    gimbalYawRawDeg = 10.0, gimbalYawBase = "aircraft",
                    smartObliquePoses = listOf(SmartObliquePose(-10.0, 30.0, 0.0)),
                )
            ),
            missionAreaWgs84 = listOf(doubleArrayOf(5.0, 53.0, 0.0)),
        )
        val json = intent.toJsonString()
        // Field names must match the Python schema verbatim (consumed by
        // flight_planner.mission_intent.intent_dict_to_imported_kmz).
        assertTrue(json.contains("\"schema_version\":"))
        assertTrue(json.contains("\"alt_egm96\":"))
        assertTrue(json.contains("\"gimbal_pitch_deg\":"))
        assertTrue(json.contains("\"gimbal_yaw_raw_deg\":"))
        assertTrue(json.contains("\"gimbal_yaw_base\":"))
        assertTrue(json.contains("\"smart_oblique_poses\":"))
        assertTrue(json.contains("\"yaw_offset_deg\":"))
        assertTrue(json.contains("\"mission_area_wgs84\":"))
    }

    private fun buildKmz(entries: Map<String, ByteArray>): ByteArray {
        val baos = ByteArrayOutputStream()
        ZipOutputStream(baos).use { zos ->
            for ((name, data) in entries) {
                zos.putNextEntry(ZipEntry(name))
                zos.write(data)
                zos.closeEntry()
            }
        }
        return baos.toByteArray()
    }
}
