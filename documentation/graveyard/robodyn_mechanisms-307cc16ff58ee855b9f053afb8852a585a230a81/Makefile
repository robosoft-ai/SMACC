# Default for EXTRA_CMAKE_FLAGS
$(eval EXTRA_CMAKE_FLAGS := -DCAPTURE_CODE_COVERAGE=OFF)

# Include with default Makefile
include $(shell rospack find mk)/cmake.mk

enableCoverage:
	$(eval EXTRA_CMAKE_FLAGS := -DCAPTURE_CODE_COVERAGE=ON)

testWithCoverage: enableCoverage test

spotless: clean
	rm -rf lib bin
	rm -rf cpp_stats
	rm -f *log4cpp.log
