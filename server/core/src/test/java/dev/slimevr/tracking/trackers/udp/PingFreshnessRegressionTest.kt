package dev.slimevr.tracking.trackers.udp

import dev.slimevr.tracking.trackers.Tracker
import dev.slimevr.tracking.trackers.TrackerStatus
import io.github.axisangles.ktmath.Quaternion
import org.junit.jupiter.api.Test
import java.net.DatagramPacket
import java.net.InetAddress
import java.net.InetSocketAddress
import java.util.function.Consumer
import kotlin.test.assertEquals

class PingFreshnessRegressionTest {
	@Test
	fun pingRepliesDoNotRefreshRotationArrivalTime() {
		val address = InetAddress.getLoopbackAddress()
		val device = UDPDevice(InetSocketAddress(address, 6969), address, "test-device")
		val tracker = Tracker(device, 42, "test", trackerPosition = null, hasRotation = true, trackRotDirection = false).apply { status = TrackerStatus.OK }
		tracker.setRotation(Quaternion.IDENTITY)
		tracker.dataTick(123_000_000L)
		device.trackers[0] = tracker
		device.lastPingPacketId = 7
		device.lastPingPacketTime = System.currentTimeMillis()
		// Construct but do not start the server: this invokes the real handler without opening a socket.
		val server = TrackersUDPServer(0, "ping-test", Consumer<Tracker> {})
		val method = TrackersUDPServer::class.java.getDeclaredMethod("processPacket", DatagramPacket::class.java, UDPPacket::class.java, UDPDevice::class.java)
		method.isAccessible = true
		repeat(3) { method.invoke(server, DatagramPacket(ByteArray(1), 1), UDPPacket10PingPong(7), device) }
		assertEquals(123_000_000L, tracker.lastRotationUpdateNanos)
		assertEquals(Quaternion.IDENTITY, tracker.getRawRotation())
	}
}
