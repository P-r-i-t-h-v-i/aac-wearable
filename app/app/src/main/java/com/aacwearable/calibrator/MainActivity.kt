package com.aacwearable.calibrator

import android.Manifest
import android.bluetooth.*
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanFilter
import android.bluetooth.le.ScanResult
import android.bluetooth.le.ScanSettings
import android.content.pm.PackageManager
import android.media.AudioFormat
import android.media.AudioRecord
import android.media.MediaRecorder
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.view.LayoutInflater
import android.view.View
import android.widget.*
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import java.io.ByteArrayOutputStream
import java.util.UUID
import kotlin.concurrent.thread

class MainActivity : AppCompatActivity() {

    companion object {
        val SERVICE_UUID: UUID = UUID.fromString("a5e7c000-9c3f-4a4e-8e1e-1a2b3c4d5e00")
        val COMMAND_CHAR_UUID: UUID = UUID.fromString("a5e7c000-9c3f-4a4e-8e1e-1a2b3c4d5e01")
        val STATUS_CHAR_UUID: UUID = UUID.fromString("a5e7c000-9c3f-4a4e-8e1e-1a2b3c4d5e02")
        val AUDIO_CHAR_UUID: UUID = UUID.fromString("a5e7c000-9c3f-4a4e-8e1e-1a2b3c4d5e03")
        val CCCD_UUID: UUID = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")

        const val DEVICE_NAME = "AAC-Wearable"
        const val SCAN_TIMEOUT_MS = 10000L

        // Must match the firmware: 8kHz mono PCM16, 2.5s max per clip
        const val SAMPLE_RATE = 8000
        const val MAX_CLIP_BYTES = 40000
        const val RECORD_MS = 2500L

        val MESSAGES = listOf("PAIN", "HUNGER", "TOILET", "SLEEP", "SEIZURE", "YES", "NO")
        // Pain and Seizure use fixed detectors in firmware, not recorded templates
        val FIXED_GESTURES = mapOf(0 to "two taps", 4 to "shake 3s")
    }

    private lateinit var connectionStatus: TextView
    private lateinit var statusLog: TextView
    private lateinit var statusScroll: ScrollView
    private lateinit var connectButton: Button
    private lateinit var messageContainer: LinearLayout
    private lateinit var uploadProgress: ProgressBar

    private val rowStates = mutableListOf<TextView>()

    private var bluetoothGatt: BluetoothGatt? = null
    private var commandChar: BluetoothGattCharacteristic? = null
    private var audioDataChar: BluetoothGattCharacteristic? = null
    private var isScanning = false
    private var mtuPayload = 20
    private val mainHandler = Handler(Looper.getMainLooper())

    // Upload state
    private var uploadChunks: List<ByteArray> = emptyList()
    private var uploadIndex = 0
    private var uploading = false
    private var awaitingEraseDone = false

    // Recording state
    private var recorder: AudioRecord? = null
    private var recording = false

    private val requiredPermissions: Array<String>
        get() {
            val base = mutableListOf(Manifest.permission.RECORD_AUDIO)
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
                base += Manifest.permission.BLUETOOTH_SCAN
                base += Manifest.permission.BLUETOOTH_CONNECT
            } else {
                base += Manifest.permission.BLUETOOTH
                base += Manifest.permission.BLUETOOTH_ADMIN
                base += Manifest.permission.ACCESS_FINE_LOCATION
            }
            return base.toTypedArray()
        }

    private val permissionLauncher =
        registerForActivityResult(ActivityResultContracts.RequestMultiplePermissions()) { results ->
            if (results.values.all { it }) startScan()
            else log("[ERROR] Permissions are required (Bluetooth + microphone).")
        }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        connectionStatus = findViewById(R.id.connectionStatus)
        statusLog = findViewById(R.id.statusLog)
        statusScroll = findViewById(R.id.statusScroll)
        connectButton = findViewById(R.id.connectButton)
        messageContainer = findViewById(R.id.messageContainer)
        uploadProgress = findViewById(R.id.uploadProgress)

        connectButton.setOnClickListener { onConnectClicked() }
        buildMessageRows()
    }

    private fun buildMessageRows() {
        val inflater = LayoutInflater.from(this)
        MESSAGES.forEachIndexed { slot, name ->
            val row = inflater.inflate(R.layout.message_row, messageContainer, false)
            row.findViewById<TextView>(R.id.msgName).text = name

            val state = row.findViewById<TextView>(R.id.msgState)
            val fixed = FIXED_GESTURES[slot]
            state.text = if (fixed != null) "gesture: $fixed" else "no gesture yet"
            rowStates.add(state)

            val gestureBtn = row.findViewById<Button>(R.id.btnGesture)
            if (fixed != null) {
                // Firmware detects these directly; there is no template to record
                gestureBtn.isEnabled = false
                gestureBtn.text = "Fixed"
            } else {
                gestureBtn.setOnClickListener { sendCommand("RECG,$slot") }
            }

            val voiceBtn = row.findViewById<Button>(R.id.btnVoice)
            voiceBtn.setOnClickListener { onVoiceClicked(slot, voiceBtn) }

            row.findViewById<Button>(R.id.btnPlay).setOnClickListener { sendCommand("PLAY,$slot") }
            row.findViewById<Button>(R.id.btnDelete).setOnClickListener { sendCommand("DEL,$slot") }

            messageContainer.addView(row)
        }
    }

    // ---------------- Voice recording ----------------

    private fun onVoiceClicked(slot: Int, button: Button) {
        if (recording) return
        if (bluetoothGatt == null) {
            Toast.makeText(this, "Connect to the device first", Toast.LENGTH_SHORT).show()
            return
        }
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.RECORD_AUDIO)
            != PackageManager.PERMISSION_GRANTED
        ) {
            permissionLauncher.launch(requiredPermissions)
            return
        }
        recordClip(slot, button)
    }

    private fun recordClip(slot: Int, button: Button) {
        val minBuf = AudioRecord.getMinBufferSize(
            SAMPLE_RATE, AudioFormat.CHANNEL_IN_MONO, AudioFormat.ENCODING_PCM_16BIT
        )
        if (minBuf <= 0) {
            log("[ERROR] 8kHz recording not supported on this device")
            return
        }

        val rec = try {
            AudioRecord(
                MediaRecorder.AudioSource.MIC, SAMPLE_RATE,
                AudioFormat.CHANNEL_IN_MONO, AudioFormat.ENCODING_PCM_16BIT,
                maxOf(minBuf, 4096)
            )
        } catch (e: SecurityException) {
            log("[ERROR] Microphone permission denied")
            return
        }

        if (rec.state != AudioRecord.STATE_INITIALIZED) {
            log("[ERROR] Could not open microphone")
            rec.release()
            return
        }

        recorder = rec
        recording = true
        button.text = "REC..."
        log("Recording ${MESSAGES[slot]} (2.5s)...")

        thread {
            val out = ByteArrayOutputStream()
            val buf = ByteArray(2048)
            rec.startRecording()
            val deadline = System.currentTimeMillis() + RECORD_MS

            while (System.currentTimeMillis() < deadline && out.size() < MAX_CLIP_BYTES) {
                val n = rec.read(buf, 0, buf.size)
                if (n > 0) {
                    val room = MAX_CLIP_BYTES - out.size()
                    out.write(buf, 0, minOf(n, room))
                }
            }

            rec.stop()
            rec.release()
            recorder = null
            recording = false

            val pcm = normalize(out.toByteArray())

            runOnUiThread {
                button.text = "Voice"
                log("Recorded ${pcm.size} bytes, uploading...")
                uploadClip(slot, pcm)
            }
        }
    }

    // Scale the clip so its loudest peak lands near full scale, matching how the
    // device's earlier baked-in audio was prepared.
    private fun normalize(pcm: ByteArray): ByteArray {
        val samples = ShortArray(pcm.size / 2)
        for (i in samples.indices) {
            samples[i] = ((pcm[i * 2].toInt() and 0xFF) or (pcm[i * 2 + 1].toInt() shl 8)).toShort()
        }
        var peak = 1
        for (s in samples) {
            val a = kotlin.math.abs(s.toInt())
            if (a > peak) peak = a
        }
        val gain = (32000.0 / peak).coerceAtMost(8.0)
        val out = ByteArray(pcm.size)
        for (i in samples.indices) {
            val v = (samples[i] * gain).toInt().coerceIn(-32768, 32767)
            out[i * 2] = (v and 0xFF).toByte()
            out[i * 2 + 1] = ((v shr 8) and 0xFF).toByte()
        }
        return out
    }

    // ---------------- BLE upload ----------------

    private fun uploadClip(slot: Int, pcm: ByteArray) {
        if (bluetoothGatt == null || audioDataChar == null) {
            log("[ERROR] Audio channel unavailable")
            return
        }
        if (uploading) {
            log("[ERROR] Another upload is in progress")
            return
        }

        val chunkSize = mtuPayload.coerceIn(20, 244)
        uploadChunks = pcm.asIterable().chunked(chunkSize) { it.toByteArray() }
        uploadIndex = 0
        uploading = true
        awaitingEraseDone = true

        uploadProgress.visibility = View.VISIBLE
        uploadProgress.progress = 0

        sendCommand("AUDIO,$slot,${pcm.size}")
        // The device erases its flash slot one page at a time (each ~85ms,
        // spread across loop() iterations to avoid blocking BLE long enough
        // to disconnect) and sends "[UP] erased, ready to receive" when done.
        // Start sending chunks on that signal; the delayed fallback below only
        // covers a dropped/garbled status notification.
        mainHandler.postDelayed({
            if (awaitingEraseDone) {
                awaitingEraseDone = false
                sendNextChunk()
            }
        }, 3000)
    }

    private fun sendNextChunk() {
        val gatt = bluetoothGatt
        val char = audioDataChar
        if (gatt == null || char == null || !uploading) return

        if (uploadIndex >= uploadChunks.size) {
            uploading = false
            sendCommand("AUDIOEND")
            runOnUiThread {
                uploadProgress.visibility = View.GONE
                log("Upload complete.")
                mainHandler.postDelayed({ sendCommand("LIST") }, 500)
            }
            return
        }

        val chunk = uploadChunks[uploadIndex]
        @Suppress("DEPRECATION")
        char.writeType = BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE
        @Suppress("DEPRECATION")
        char.value = chunk
        @Suppress("DEPRECATION")
        val ok = checkPermAndBool { gatt.writeCharacteristic(char) }

        if (!ok) {
            // Stack busy - retry shortly rather than dropping the chunk
            mainHandler.postDelayed({ sendNextChunk() }, 20)
            return
        }

        uploadIndex++
        val pct = uploadIndex * 100 / uploadChunks.size
        runOnUiThread { uploadProgress.progress = pct }
    }

    // ---------------- BLE plumbing ----------------

    private fun onConnectClicked() {
        if (bluetoothGatt != null) {
            checkPermAnd { bluetoothGatt?.disconnect() }
            return
        }
        if (hasPermissions()) startScan() else permissionLauncher.launch(requiredPermissions)
    }

    private fun hasPermissions(): Boolean = requiredPermissions.all {
        ContextCompat.checkSelfPermission(this, it) == PackageManager.PERMISSION_GRANTED
    }

    private fun getBluetoothAdapter(): BluetoothAdapter? =
        (getSystemService(BLUETOOTH_SERVICE) as? BluetoothManager)?.adapter

    private fun startScan() {
        val adapter = getBluetoothAdapter()
        if (adapter == null || !adapter.isEnabled) {
            log("[ERROR] Bluetooth is off. Please enable it.")
            return
        }
        val scanner = adapter.bluetoothLeScanner ?: run {
            log("[ERROR] BLE scanner unavailable."); return
        }

        log("Scanning for $DEVICE_NAME ...")
        connectionStatus.text = "Scanning..."
        connectionStatus.setTextColor(0xFFFF6D00.toInt())

        val filters = listOf(
            ScanFilter.Builder().setServiceUuid(android.os.ParcelUuid(SERVICE_UUID)).build()
        )
        val settings = ScanSettings.Builder()
            .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY).build()

        isScanning = true
        checkPermAnd { scanner.startScan(filters, settings, scanCallback) }

        mainHandler.postDelayed({
            if (isScanning) {
                stopScan()
                log("Scan timed out. Is the device powered on and nearby?")
                connectionStatus.text = "Disconnected"
                connectionStatus.setTextColor(0xFFB00020.toInt())
            }
        }, SCAN_TIMEOUT_MS)
    }

    private fun stopScan() {
        if (!isScanning) return
        isScanning = false
        val scanner = getBluetoothAdapter()?.bluetoothLeScanner ?: return
        checkPermAnd { scanner.stopScan(scanCallback) }
    }

    private val scanCallback = object : ScanCallback() {
        override fun onScanResult(callbackType: Int, result: ScanResult) {
            stopScan()
            log("Found device, connecting...")
            checkPermAnd {
                bluetoothGatt = result.device.connectGatt(this@MainActivity, false, gattCallback)
            }
        }

        override fun onScanFailed(errorCode: Int) {
            isScanning = false
            log("[ERROR] Scan failed (code $errorCode)")
        }
    }

    private val gattCallback = object : BluetoothGattCallback() {
        override fun onConnectionStateChange(gatt: BluetoothGatt, status: Int, newState: Int) {
            if (newState == BluetoothProfile.STATE_CONNECTED) {
                runOnUiThread {
                    connectionStatus.text = "Connected - negotiating..."
                    connectionStatus.setTextColor(0xFFFF6D00.toInt())
                }
                // Default 23-byte MTU truncates status text and cripples upload speed
                checkPermAnd { gatt.requestMtu(247) }
            } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                uploading = false
                runOnUiThread {
                    connectionStatus.text = "Disconnected"
                    connectionStatus.setTextColor(0xFFB00020.toInt())
                    connectButton.text = "Connect to Device"
                    uploadProgress.visibility = View.GONE
                    log("Disconnected.")
                }
                bluetoothGatt = null
                commandChar = null
                audioDataChar = null
            }
        }

        override fun onMtuChanged(gatt: BluetoothGatt, mtu: Int, status: Int) {
            mtuPayload = (mtu - 3).coerceIn(20, 244)
            log("MTU $mtu (payload $mtuPayload)")
            checkPermAnd { gatt.discoverServices() }
        }

        override fun onServicesDiscovered(gatt: BluetoothGatt, status: Int) {
            if (status != BluetoothGatt.GATT_SUCCESS) {
                log("[ERROR] Service discovery failed."); return
            }
            val service = gatt.getService(SERVICE_UUID) ?: run {
                log("[ERROR] Service not found on device."); return
            }

            commandChar = service.getCharacteristic(COMMAND_CHAR_UUID)
            audioDataChar = service.getCharacteristic(AUDIO_CHAR_UUID)
            val statusCharacteristic = service.getCharacteristic(STATUS_CHAR_UUID)

            if (statusCharacteristic != null) {
                checkPermAnd { gatt.setCharacteristicNotification(statusCharacteristic, true) }
                statusCharacteristic.getDescriptor(CCCD_UUID)?.let { d ->
                    @Suppress("DEPRECATION")
                    d.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
                    @Suppress("DEPRECATION")
                    checkPermAnd { gatt.writeDescriptor(d) }
                }
            }

            runOnUiThread {
                connectionStatus.text = "Connected to $DEVICE_NAME"
                connectionStatus.setTextColor(0xFF00C853.toInt())
                connectButton.text = "Disconnect"
                log("Ready.")
            }
            mainHandler.postDelayed({ sendCommand("LIST") }, 800)
        }

        override fun onCharacteristicWrite(
            gatt: BluetoothGatt, characteristic: BluetoothGattCharacteristic, status: Int
        ) {
            // Drives upload flow control: next chunk only once the stack accepts this one
            if (characteristic.uuid == AUDIO_CHAR_UUID && uploading) {
                sendNextChunk()
            }
        }

        @Suppress("DEPRECATION")
        override fun onCharacteristicChanged(
            gatt: BluetoothGatt, characteristic: BluetoothGattCharacteristic
        ) {
            val text = characteristic.getStringValue(0) ?: return
            log(text)
            updateRowFromStatus(text)

            if (awaitingEraseDone && text.contains("erased, ready")) {
                awaitingEraseDone = false
                sendNextChunk()
            }
        }
    }

    // Firmware LIST lines look like: "1 HUNGER voice:YES gesture:--"
    private fun updateRowFromStatus(text: String) {
        val parts = text.trim().split(" ")
        if (parts.size < 4) return
        val slot = parts[0].toIntOrNull() ?: return
        if (slot !in MESSAGES.indices) return
        val voice = parts.firstOrNull { it.startsWith("voice:") }?.removePrefix("voice:") ?: return
        val gesture = parts.firstOrNull { it.startsWith("gesture:") }?.removePrefix("gesture:") ?: return

        runOnUiThread {
            val v = if (voice == "YES") "voice ✓" else "no voice"
            val g = when {
                FIXED_GESTURES.containsKey(slot) -> "gesture: ${FIXED_GESTURES[slot]}"
                gesture == "YES" -> "gesture ✓"
                else -> "no gesture"
            }
            rowStates[slot].text = "$v · $g"
        }
    }

    private fun sendCommand(cmd: String) {
        val gatt = bluetoothGatt
        val char = commandChar
        if (gatt == null || char == null) {
            Toast.makeText(this, "Not connected", Toast.LENGTH_SHORT).show()
            return
        }
        @Suppress("DEPRECATION")
        char.setValue(cmd)
        @Suppress("DEPRECATION")
        checkPermAnd { gatt.writeCharacteristic(char) }
        log("> $cmd")
    }

    private fun checkPermAnd(action: () -> Unit) {
        checkPermAndBool { action(); true }
    }

    private fun checkPermAndBool(action: () -> Boolean): Boolean {
        val granted = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT) ==
                PackageManager.PERMISSION_GRANTED
        } else true

        if (!granted) {
            log("[ERROR] Missing Bluetooth permission.")
            return false
        }
        return try {
            action()
        } catch (e: SecurityException) {
            log("[ERROR] Permission denied: ${e.message}")
            false
        }
    }

    private fun log(message: String) {
        runOnUiThread {
            statusLog.append("$message\n")
            statusScroll.post { statusScroll.fullScroll(View.FOCUS_DOWN) }
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        stopScan()
        recorder?.release()
        checkPermAnd { bluetoothGatt?.close() }
    }
}
