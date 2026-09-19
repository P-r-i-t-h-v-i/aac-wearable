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
import android.widget.Button
import android.widget.ScrollView
import android.widget.TextView
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
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
    }

    private lateinit var connectionStatus: TextView
    private lateinit var statusLog: TextView
    private lateinit var statusScroll: ScrollView
    private lateinit var connectButton: Button

    private var bluetoothGatt: BluetoothGatt? = null
    private var commandChar: BluetoothGattCharacteristic? = null
    private var isScanning = false
    private val mainHandler = Handler(Looper.getMainLooper())

    private val requiredPermissions: Array<String>
        get() = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            arrayOf(Manifest.permission.BLUETOOTH_SCAN, Manifest.permission.BLUETOOTH_CONNECT)
        } else {
            arrayOf(
                Manifest.permission.BLUETOOTH,
                Manifest.permission.BLUETOOTH_ADMIN,
                Manifest.permission.ACCESS_FINE_LOCATION
            )
        }

    private val permissionLauncher =
        registerForActivityResult(androidx.activity.result.contract.ActivityResultContracts.RequestMultiplePermissions()) { results ->
            if (results.values.all { it }) {
                startScan()
            } else {
                log("[ERROR] Bluetooth/location permissions are required to connect.")
            }
        }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        connectionStatus = findViewById(R.id.connectionStatus)
        statusLog = findViewById(R.id.statusLog)
        statusScroll = findViewById(R.id.statusScroll)
        connectButton = findViewById(R.id.connectButton)

        connectButton.setOnClickListener { onConnectClicked() }

        findViewById<Button>(R.id.calHeadButton).setOnClickListener { sendCommand("CAL1") }
        findViewById<Button>(R.id.calChestButton).setOnClickListener { sendCommand("CAL2") }
        findViewById<Button>(R.id.calStomachButton).setOnClickListener { sendCommand("CAL3") }
        findViewById<Button>(R.id.calRestButton).setOnClickListener { sendCommand("CAL0") }
        findViewById<Button>(R.id.showButton).setOnClickListener { sendCommand("SHOW") }
        findViewById<Button>(R.id.testHeadButton).setOnClickListener { sendCommand("H") }
        findViewById<Button>(R.id.testChestButton).setOnClickListener { sendCommand("C") }
        findViewById<Button>(R.id.testStomachButton).setOnClickListener { sendCommand("S") }
        findViewById<Button>(R.id.testSeizureButton).setOnClickListener { sendCommand("E") }
    }

    private fun onConnectClicked() {
        if (bluetoothGatt != null) {
            disconnect()
            return
        }
        if (hasPermissions()) {
            startScan()
        } else {
            permissionLauncher.launch(requiredPermissions)
        }
    }

    private fun hasPermissions(): Boolean =
        requiredPermissions.all {
            ContextCompat.checkSelfPermission(this, it) == PackageManager.PERMISSION_GRANTED
        }

    private fun getBluetoothAdapter(): BluetoothAdapter? {
        val manager = getSystemService(BLUETOOTH_SERVICE) as? BluetoothManager
        return manager?.adapter
    }

    private fun startScan() {
        val adapter = getBluetoothAdapter()
        if (adapter == null || !adapter.isEnabled) {
            log("[ERROR] Bluetooth is off. Please enable it.")
            return
        }
        val scanner = adapter.bluetoothLeScanner ?: run {
            log("[ERROR] BLE scanner unavailable.")
            return
        }

        log("Scanning for $DEVICE_NAME ...")
        connectionStatus.text = "Scanning..."
        connectionStatus.setTextColor(0xFFFF6D00.toInt())

        val filters = listOf(
            ScanFilter.Builder().setServiceUuid(android.os.ParcelUuid(SERVICE_UUID)).build()
        )
        val settings = ScanSettings.Builder()
            .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
            .build()

        isScanning = true
        checkPermAnd { scanner.startScan(filters, settings, scanCallback) }

        mainHandler.postDelayed({
            if (isScanning) {
                stopScan()
                log("Scan timed out. Make sure the device is powered on and nearby.")
                connectionStatus.text = "Disconnected"
                connectionStatus.setTextColor(0xFFB00020.toInt())
            }
        }, SCAN_TIMEOUT_MS)
    }

    private fun stopScan() {
        if (!isScanning) return
        isScanning = false
        val adapter = getBluetoothAdapter() ?: return
        val scanner = adapter.bluetoothLeScanner ?: return
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
                    connectionStatus.text = "Connected - discovering services..."
                    connectionStatus.setTextColor(0xFFFF6D00.toInt())
                }
                // Default BLE ATT MTU is 23 bytes (20 usable), which truncates our
                // status notifications. Request a larger MTU before discovering
                // services; onMtuChanged (below) continues from there either way.
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
            if (status == BluetoothGatt.GATT_SUCCESS) {
                log("MTU negotiated: $mtu bytes")
            } else {
                log("MTU negotiation failed, continuing with default (status $status)")
            }
            checkPermAnd { gatt.discoverServices() }
        }

        override fun onServicesDiscovered(gatt: BluetoothGatt, status: Int) {
            if (status != BluetoothGatt.GATT_SUCCESS) {
                log("[ERROR] Service discovery failed.")
                return
            }
            val service = gatt.getService(SERVICE_UUID)
            if (service == null) {
                log("[ERROR] Calibration service not found on device.")
                return
            }
            commandChar = service.getCharacteristic(COMMAND_CHAR_UUID)
            val statusCharacteristic = service.getCharacteristic(STATUS_CHAR_UUID)

            if (statusCharacteristic != null) {
                checkPermAnd { gatt.setCharacteristicNotification(statusCharacteristic, true) }
                val descriptor = statusCharacteristic.getDescriptor(CCCD_UUID)
                if (descriptor != null) {
                    descriptor.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
                    checkPermAnd { gatt.writeDescriptor(descriptor) }
                }
            }

            runOnUiThread {
                connectionStatus.text = "Connected to $DEVICE_NAME"
                connectionStatus.setTextColor(0xFF00C853.toInt())
                connectButton.text = "Disconnect"
                log("Ready. Hold the device where you want each part and tap the matching button.")
            }
        }

        @Suppress("DEPRECATION")
        override fun onCharacteristicChanged(gatt: BluetoothGatt, characteristic: BluetoothGattCharacteristic) {
            val text = characteristic.getStringValue(0) ?: return
            log(text)
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
        checkPermAnd {
            @Suppress("DEPRECATION")
            gatt.writeCharacteristic(char)
        }
        log("> $cmd")
    }

    private fun disconnect() {
        checkPermAnd { bluetoothGatt?.disconnect() }
    }

    private fun checkPermAnd(action: () -> Unit) {
        val needsConnect = Build.VERSION.SDK_INT >= Build.VERSION_CODES.S
        val granted = if (needsConnect) {
            ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT) ==
                PackageManager.PERMISSION_GRANTED &&
                ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_SCAN) ==
                PackageManager.PERMISSION_GRANTED
        } else {
            true
        }
        if (granted) {
            try {
                action()
            } catch (e: SecurityException) {
                log("[ERROR] Permission denied: ${e.message}")
            }
        } else {
            log("[ERROR] Missing Bluetooth permission.")
        }
    }

    private fun log(message: String) {
        runOnUiThread {
            statusLog.append("$message\n")
            statusScroll.post { statusScroll.fullScroll(android.view.View.FOCUS_DOWN) }
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        stopScan()
        checkPermAnd { bluetoothGatt?.close() }
    }
}
