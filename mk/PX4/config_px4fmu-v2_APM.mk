#
# Makefile for the px4fmu-v2_APM configuration
#
include $(SKETCHBOOK)/mk/PX4/px4_common.mk

MODULES		+= drivers/boards/px4fmu-v2
MODULES		+= drivers/pwm_input
<<<<<<< HEAD
=======
#MODULES     += drivers/hz16wa
#MODULES		+= drivers/m006
MODULES         += modules/uavcan
MODULES         += lib/mathlib
>>>>>>> 2430caca065e293b45b204e891ae1bd3cc86dab2
MODULES		+= drivers/px4io
MODULES		+= modules/uavcan
