.PHONY: set_hardware_v1 set_hardware_v2

set_hardware_v1:
	if [ -f sdkconfig ]; then rm sdkconfig; fi
	cp manifests/sdkconfig/hardware_v1.sdkconfig sdkconfig
	sed -i 's/bno08x/hi229/g' CMakeLists.txt

set_hardware_v2:
	if [ -f sdkconfig ]; then rm sdkconfig; fi
	cp manifests/sdkconfig/hardware_v2.sdkconfig sdkconfig
	sed -i 's/hi229/bno08x/g' CMakeLists.txt
