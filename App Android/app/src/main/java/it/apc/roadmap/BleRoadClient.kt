package it.apc.appmappa

import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCallback
import android.bluetooth.BluetoothGattCharacteristic
import android.bluetooth.BluetoothGattDescriptor
import android.bluetooth.BluetoothManager
import android.bluetooth.le.BluetoothLeScanner
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanResult
import android.content.Context
import android.os.Build
import java.util.UUID

class BleRoadClient(
    context: Context,
    private val onStatus: (String) -> Unit,
    private val onLine: (String) -> Unit
) {
    companion object {
        private const val DEVICE_NAME = "HC-05"
        private val UART_UUID: UUID = UUID.fromString("0000ffe1-0000-1000-8000-00805f9b34fb")
        private val CCCD_UUID: UUID = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")
    }

    private val appContext = context.applicationContext
    private val adapter: BluetoothAdapter? = (appContext.getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager).adapter
    private var scanner: BluetoothLeScanner? = null
    private var gatt: BluetoothGatt? = null
    private val bytes = StringBuilder()

    @SuppressLint("MissingPermission")
    fun connect() {
        close()
        val localAdapter = adapter
        if (localAdapter == null || !localAdapter.isEnabled) {
            onStatus("Bluetooth spento o non disponibile")
            return
        }
        scanner = localAdapter.bluetoothLeScanner
        onStatus("Ricerca BLE di $DEVICE_NAME…")
        scanner?.startScan(scanCallback)
    }

    @SuppressLint("MissingPermission")
    fun close() {
        scanner?.stopScan(scanCallback)
        scanner = null
        gatt?.close()
        gatt = null
        bytes.clear()
    }

    @SuppressLint("MissingPermission")
    private val scanCallback = object : ScanCallback() {
        override fun onScanResult(callbackType: Int, result: ScanResult) {
            val name = result.device.name ?: result.scanRecord?.deviceName ?: ""
            if (!name.equals(DEVICE_NAME, ignoreCase = true)) return
            scanner?.stopScan(this)
            onStatus("Connessione a $DEVICE_NAME…")
            gatt = result.device.connectGatt(appContext, false, gattCallback, BluetoothDevice.TRANSPORT_LE)
        }

        override fun onScanFailed(errorCode: Int) {
            onStatus("Errore scansione BLE: $errorCode")
        }
    }

    private val gattCallback = object : BluetoothGattCallback() {
        @SuppressLint("MissingPermission")
        override fun onConnectionStateChange(gatt: BluetoothGatt, status: Int, newState: Int) {
            if (status != BluetoothGatt.GATT_SUCCESS) {
                onStatus("BLE disconnesso (errore $status)")
                gatt.close()
                return
            }
            if (newState == android.bluetooth.BluetoothProfile.STATE_CONNECTED) {
                onStatus("BLE connesso: ricerca FFE1…")
                gatt.discoverServices()
            } else {
                onStatus("BLE disconnesso")
                gatt.close()
            }
        }

        @SuppressLint("MissingPermission")
        override fun onServicesDiscovered(gatt: BluetoothGatt, status: Int) {
            // Il clone chiamato HC-05 spesso espone FFE1 nel servizio FFE0;
            // cerchiamo quindi la caratteristica in tutti i servizi GATT.
            val characteristic = gatt.services
                .asSequence()
                .mapNotNull { service -> service.getCharacteristic(UART_UUID) }
                .firstOrNull()
            if (status != BluetoothGatt.GATT_SUCCESS || characteristic == null) {
                onStatus("Caratteristica FFE1 non trovata")
                return
            }
            gatt.setCharacteristicNotification(characteristic, true)
            val descriptor = characteristic.getDescriptor(CCCD_UUID)
            if (descriptor == null) {
                onStatus("Descrittore notifiche BLE non trovato")
                return
            }
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
                gatt.writeDescriptor(descriptor, BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE)
            } else {
                @Suppress("DEPRECATION")
                run { descriptor.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE; gatt.writeDescriptor(descriptor) }
            }
        }

        override fun onDescriptorWrite(gatt: BluetoothGatt, descriptor: BluetoothGattDescriptor, status: Int) {
            onStatus(if (status == BluetoothGatt.GATT_SUCCESS) "BLE pronto: notifiche attive" else "Errore notifiche BLE: $status")
        }

        @Deprecated("Deprecated in Java")
        override fun onCharacteristicChanged(gatt: BluetoothGatt, characteristic: BluetoothGattCharacteristic) {
            acceptBytes(characteristic.value)
        }

        override fun onCharacteristicChanged(gatt: BluetoothGatt, characteristic: BluetoothGattCharacteristic, value: ByteArray) {
            acceptBytes(value)
        }
    }

    private fun acceptBytes(value: ByteArray) {
        bytes.append(value.toString(Charsets.UTF_8))
        while (true) {
            val newline = bytes.indexOf("\n")
            if (newline < 0) return
            val line = bytes.substring(0, newline).trim()
            bytes.delete(0, newline + 1)
            if (line.isNotEmpty()) onLine(line)
        }
    }
}
