package com.aacwearable.calibrator

import android.Manifest
import android.bluetooth.*
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanFilter
import android.bluetooth.le.ScanResult
import android.bluetooth.le.ScanSettings
import android.content.pm.PackageManager
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
import java.util.UUID

class MainActivity : AppCompatActivity() {

    companion object {
        val SERVICE_UUID: UUID = UUID.fromString("a5e7c000-9c3f-4a4e-8e1e-1a2b3c4d5e00")
        val COMMAND_CHAR_UUID: UUID = UUID.fromString("a5e7c000-9c3f-4a4e-8e1e-1a2b3c4d5e01")
        val STATUS_CHAR_UUID: UUID = UUID.fromString("a5e7c000-9c3f-4a4e-8e1e-1a2b3c4d5e02")
        val CCCD_UUID: UUID = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")

        const val DEVICE_NAME = "AAC-Wearable"
        const val SCAN_TIMEOUT_MS = 10000L

        val MESSAGES = listOf("REST", "HUNGER", "TOILET", "SLEEP", "SEIZURE", "YES", "NO")
        // Pain and Seizure use fixed detectors in firmware, not recorded templates
        val FIXED_GESTURES = mapOf(4 to "shake 3s")
    }

    private lateinit var connectionStatus: TextView
    private lateinit var statusLog: TextView
    private lateinit var statusScroll: ScrollView
    private lateinit var connectButton: Button
    private lateinit var messageContainer: LinearLayout
    private lateinit var accuracyText: TextView

    private val rowStates = mutableListOf<TextView>()

    private var bluetoothGatt: BluetoothGatt? = null
    private var commandChar: BluetoothGattCharacteristic? = null
    private var isScanning = false
    private var mtuPayload = 20
    private val mainHandler = Handler(Looper.getMainLooper())


    private val requiredPermissions: Array<String>
        get() {
            val base = mutableListOf<String>()
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
            else log("[ERROR] Bluetooth permissions are required.")
        }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        connectionStatus = findViewById(R.id.connectionStatus)
        statusLog = findViewById(R.id.statusLog)
        statusScroll = findViewById(R.id.statusScroll)
        connectButton = findViewById(R.id.connectButton)
        messageContainer = findViewById(R.id.messageContainer)
        accuracyText = findViewById(R.id.accuracyText)
        findViewById<Button>(R.id.accuracyButton).setOnClickListener { sendCommand("ACC") }

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
                gestureBtn.text = "Train x10"
                gestureBtn.setOnClickListener { sendCommand("RECG,$slot") }
            }

            val voiceBtn = row.findViewById<Button>(R.id.btnVoice)
            voiceBtn.setOnClickListener { sendCommand("RECV,$slot") }

            row.findViewById<Button>(R.id.btnPlay).setOnClickListener { sendCommand("PLAY,$slot") }
            row.findViewById<Button>(R.id.btnDelete).setOnClickListener { sendCommand("DEL,$slot") }

            messageContainer.addView(row)
        }
    }

    // ---------------- Voice recording ----------------

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
                runOnUiThread {
                    connectionStatus.text = "Disconnected"
                    connectionStatus.setTextColor(0xFFB00020.toInt())
                    connectButton.text = "Connect to Device"
                    log("Disconnected.")
                }
                bluetoothGatt = null
                commandChar = null
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

        @Suppress("DEPRECATION")
        override fun onCharacteristicChanged(
            gatt: BluetoothGatt, characteristic: BluetoothGattCharacteristic
        ) {
            val text = characteristic.getStringValue(0) ?: return
            log(text)
            updateRowFromStatus(text)
            updateAccuracy(text)
        }
    }

    // Firmware accuracy lines: "[ACC] HUNGER 9/10 (90%)", "[ACC] OVERALL 57/60 = 95.0%"
    private fun updateAccuracy(text: String) {
        if (!text.startsWith("[ACC]")) return
        val body = text.removePrefix("[ACC]").trim()
        runOnUiThread {
            when {
                body.startsWith("checking") -> accuracyText.text = "Accuracy: checking..."
                body.startsWith("OVERALL") ->
                    accuracyText.text = "Accuracy: " + body.removePrefix("OVERALL").trim()
                body.startsWith("Train at least") -> accuracyText.text = "Accuracy: $body"
            }
        }
        // Train/delete change the gesture counts, so refresh the rows.
        if (text.contains("match limit")) mainHandler.postDelayed({ sendCommand("LIST") }, 300)
    }

    // Firmware LIST lines look like: "1 HUNGER voice:YES gesture:7/10"
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
                gesture.endsWith("/10") && !gesture.startsWith("0/") -> "gesture $gesture"
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
        checkPermAnd { bluetoothGatt?.close() }
    }
}
