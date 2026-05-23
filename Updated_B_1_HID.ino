C:\Users\USER\AppData\Local\Temp\.arduinoIDE-unsaved2026423-5288-1hru79i.z1wu\sketch_may23b\sketch_may23b.ino: In function 'void startBLE()':
C:\Users\USER\AppData\Local\Temp\.arduinoIDE-unsaved2026423-5288-1hru79i.z1wu\sketch_may23b\sketch_may23b.ino:535:14: error: 'setEncryptionLevel' is not a member of 'BLEDevice'
  535 |   BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT_NO_MITM);
      |              ^~~~~~
C:\Users\USER\AppData\Local\Temp\.arduinoIDE-unsaved2026423-5288-1hru79i.z1wu\sketch_may23b\sketch_may23b.ino:535:33: error: 'ESP_BLE_SEC_ENCRYPT_NO_MITM' was not declared in this scope
  535 |   BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT_NO_MITM);
      |                                 ^~~~~~~
C:\Users\USER\AppData\Local\Temp\.arduinoIDE-unsaved2026423-5288-1hru79i.z1wu\sketch_may23b\sketch_may23b.ino:564:39: error: 'HID_SERVICE_UUID_16' was not declared in this scope
  564 |   advData.setCompleteServices(BLEUUID(HID_SERVICE_UUID_16));
      |                                       ^~~~~~~
exit status 1

Compilation error: 'setEncryptionLevel' is not a member of 'BLEDevice'
