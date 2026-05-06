package com.aeroscan.rccompanion.mop

import org.junit.Assert.assertArrayEquals
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class AugmentFramingTest {

    @Test
    fun `header round-trips magic + version + bodyLen`() {
        val hdr = AugmentFraming.frame(MopConstants.MAGIC_AUGM, ByteArray(0))
            .copyOfRange(0, AugmentFraming.HEADER_LEN)
        val parsed = AugmentFraming.parseHeader(hdr)
        assertEquals(MopConstants.MAGIC_AUGM, parsed.magic)
        assertEquals(MopConstants.FRAME_VERSION, parsed.version)
        assertEquals(0, parsed.bodyLen)
    }

    @Test
    fun `frame is bytes-on-the-wire compatible with kmz_runner_c LE encoding`() {
        // Manually built reference: magic "AUGM" (4) + version 1 LE (4) + bodyLen 12 LE (4) + reserved 0 (4)
        val expectedHeader = byteArrayOf(
            'A'.code.toByte(), 'U'.code.toByte(), 'G'.code.toByte(), 'M'.code.toByte(),
            0x01, 0x00, 0x00, 0x00,
            0x0c, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
        )
        val body = ByteArray(12) { (it + 1).toByte() }
        val frame = AugmentFraming.frame(MopConstants.MAGIC_AUGM, body)
        assertArrayEquals(expectedHeader, frame.copyOfRange(0, 16))
        assertArrayEquals(body, frame.copyOfRange(16, 28))
    }

    @Test
    fun `AUGM body packs intent + cloud with int32 LE length prefixes`() {
        val intent = "{}".toByteArray(Charsets.UTF_8)
        val cloud = byteArrayOf(0x01, 0x02, 0x03, 0x04, 0x05)
        val body = AugmentFraming.buildAugmBody(intent, cloud)
        // 4 + 2 + 4 + 5 = 15
        assertEquals(15, body.size)
        // intent length, LE
        assertEquals(intent.size, body[0].toInt() and 0xff)
        assertEquals(0, body[1].toInt())
        assertEquals(0, body[2].toInt())
        assertEquals(0, body[3].toInt())
        // intent bytes
        assertArrayEquals(intent, body.copyOfRange(4, 4 + intent.size))
        // cloud length, LE, at offset 4 + intent.size
        val cloudLenOff = 4 + intent.size
        assertEquals(cloud.size, body[cloudLenOff].toInt() and 0xff)
        // cloud bytes
        assertArrayEquals(cloud, body.copyOfRange(cloudLenOff + 4, cloudLenOff + 4 + cloud.size))
    }

    @Test
    fun `PRVW body parser splits summary + kmz`() {
        // Build a synthetic PRVW body: summary "ABC", kmz 5 random bytes
        val summary = "ABC".toByteArray(Charsets.UTF_8)
        val kmz = byteArrayOf(0x10, 0x20, 0x30, 0x40, 0x50)
        val body = AugmentFraming.buildAugmBody(summary, kmz)  // shape is identical to PRVW

        val preview = AugmentFraming.parsePreviewBody(body)
        assertArrayEquals(summary, preview.summaryJson)
        assertArrayEquals(kmz, preview.augmentedKmz)
    }

    @Test
    fun `EXEC body packs mission id`() {
        val missionId = "20260506T201234Z_512"
        val body = AugmentFraming.buildExecBody(missionId)
        // 4-byte LE length prefix + UTF-8 mission id
        val expectedLen = missionId.toByteArray(Charsets.UTF_8).size
        assertEquals(4 + expectedLen, body.size)
        assertEquals(expectedLen, body[0].toInt() and 0xff)
        assertEquals(0, body[1].toInt())
        val payload = String(body, 4, expectedLen, Charsets.UTF_8)
        assertEquals(missionId, payload)
    }

    @Test
    fun `EXEC body with empty id is 4 bytes`() {
        val body = AugmentFraming.buildExecBody("")
        assertEquals(4, body.size)
        assertEquals(0, body[0].toInt())
    }

    @Test
    fun `parseHeader rejects bad header length`() {
        try {
            AugmentFraming.parseHeader(ByteArray(8))
            assertTrue("expected throw", false)
        } catch (e: IllegalArgumentException) {
            // expected
        }
    }
}
