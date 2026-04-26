package com.aeroscan.rccompanion.mop

import org.junit.Assert.assertArrayEquals
import org.junit.Assert.assertEquals
import org.junit.Assert.assertThrows
import org.junit.Test
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.security.MessageDigest

class FramingTest {

    @Test
    fun header_layout_matches_spec() {
        val header = Framing.buildHeader("mission.kmz", 12345L)
        val nameBytes = "mission.kmz".toByteArray(Charsets.UTF_8)
        assertEquals(Framing.HEADER_FIXED_LEN + nameBytes.size, header.size)

        val buf = ByteBuffer.wrap(header).order(ByteOrder.LITTLE_ENDIAN)
        assertEquals(MopConstants.FRAME_MAGIC, buf.int)
        assertEquals(MopConstants.FRAME_VERSION, buf.int)
        assertEquals(nameBytes.size, buf.int)
        assertEquals(12345L, buf.long)

        val tail = ByteArray(nameBytes.size).also { buf.get(it) }
        assertArrayEquals(nameBytes, tail)
    }

    @Test
    fun header_handles_unicode_filename() {
        val name = "städtische_inspektion_2026.kmz"
        val header = Framing.buildHeader(name, 0L)
        val expectedNameBytes = name.toByteArray(Charsets.UTF_8)
        val buf = ByteBuffer.wrap(header).order(ByteOrder.LITTLE_ENDIAN)
        buf.position(8)  // skip magic + version
        assertEquals(expectedNameBytes.size, buf.int)
        assertEquals(0L, buf.long)
        val nameTail = ByteArray(expectedNameBytes.size).also { buf.get(it) }
        assertArrayEquals(expectedNameBytes, nameTail)
    }

    @Test
    fun header_rejects_negative_size() {
        assertThrows(IllegalArgumentException::class.java) {
            Framing.buildHeader("x.kmz", -1L)
        }
    }

    @Test
    fun digest_matches_md5_of_streamed_bytes() {
        val payload = ByteArray(4096) { (it % 251).toByte() }
        val expected = MessageDigest.getInstance("MD5").digest(payload)

        val streamed = Framing.newDigest()
        // Simulate the streaming-write path used by MopFileSender.
        var off = 0
        while (off < payload.size) {
            val take = minOf(MopConstants.CHUNK_SIZE, payload.size - off)
            streamed.update(payload, off, take)
            off += take
        }
        val got = Framing.finishDigest(streamed)
        assertEquals(Framing.MD5_LEN, got.size)
        assertArrayEquals(expected, got)
    }

    @Test
    fun zero_length_payload_digest_matches_md5_of_empty() {
        val expected = MessageDigest.getInstance("MD5").digest(ByteArray(0))
        val got = Framing.finishDigest(Framing.newDigest())
        assertArrayEquals(expected, got)
    }
}
