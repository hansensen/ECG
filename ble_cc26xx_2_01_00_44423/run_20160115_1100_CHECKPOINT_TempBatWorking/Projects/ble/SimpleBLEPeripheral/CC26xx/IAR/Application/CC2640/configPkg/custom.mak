## THIS IS A GENERATED FILE -- DO NOT EDIT
.configuro: .libraries,rm3 linker.cmd package/cfg/appBLE_prm3.orm3

# To simplify configuro usage in makefiles:
#     o create a generic linker command file name 
#     o set modification times of compiler.opt* files to be greater than
#       or equal to the generated config header
#
linker.cmd: package/cfg/appBLE_prm3.xdl
	$(SED) 's"^\"\(package/cfg/appBLE_prm3cfg.cmd\)\"$""\"D:/run_20160115_1100_CHECKPOINT_TempBatWorking_20160312/run_20160115_1100_CHECKPOINT_TempBatWorking/Projects/ble/SimpleBLEPeripheral/CC26xx/IAR/Application/CC2640/configPkg/\1\""' package/cfg/appBLE_prm3.xdl > $@
	-$(SETDATE) -r:max package/cfg/appBLE_prm3.h compiler.opt compiler.opt.defs
