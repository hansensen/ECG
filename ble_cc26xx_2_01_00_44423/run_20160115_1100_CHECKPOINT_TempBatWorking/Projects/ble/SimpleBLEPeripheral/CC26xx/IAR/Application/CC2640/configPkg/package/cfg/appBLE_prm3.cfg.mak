# invoke SourceDir generated makefile for appBLE.prm3
appBLE.prm3: .libraries,appBLE.prm3
.libraries,appBLE.prm3: package/cfg/appBLE_prm3.xdl
	$(MAKE) -f D:\run_20160115_1100_CHECKPOINT_TempBatWorking_20160312\run_20160115_1100_CHECKPOINT_TempBatWorking\Projects\ble\SimpleBLEPeripheral\CC26xx\IAR\Config/src/makefile.libs

clean::
	$(MAKE) -f D:\run_20160115_1100_CHECKPOINT_TempBatWorking_20160312\run_20160115_1100_CHECKPOINT_TempBatWorking\Projects\ble\SimpleBLEPeripheral\CC26xx\IAR\Config/src/makefile.libs clean

