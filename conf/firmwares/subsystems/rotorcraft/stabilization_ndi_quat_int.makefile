STAB_ATT_CFLAGS  = -DSTABILIZATION_ATTITUDE_TYPE_INT
STAB_ATT_CFLAGS += -DSTABILIZATION_ATTITUDE_TYPE_H=\"stabilization/stabilization_attitude_ndi_quat_int.h\"
STAB_ATT_SRCS  = $(SRC_FIRMWARE)/stabilization/stabilization_attitude_ndi_ref_quat_int.c
STAB_ATT_SRCS += $(SRC_FIRMWARE)/stabilization/stabilization_rate_ref_int.c
STAB_ATT_SRCS += $(SRC_FIRMWARE)/stabilization/stabilization_rate_ndi_int.c
STAB_ATT_SRCS += $(SRC_FIRMWARE)/stabilization/stabilization_attitude_ndi_quat_int.c
STAB_ATT_SRCS += $(SRC_FIRMWARE)/stabilization/stabilization_attitude_quat_transformations.c
STAB_ATT_SRCS += $(SRC_FIRMWARE)/stabilization/stabilization_attitude_rc_setpoint.c

STAB_ATT_SRCS += $(SRC_FIRMWARE)/stabilization/stabilization_altitude_ndi_quat_int.c
STAB_ATT_SRCS += $(SRC_FIRMWARE)/stabilization/stabilization_attitude_ndi_pch_model.c

ap.CFLAGS += $(STAB_ATT_CFLAGS)
ap.srcs += $(STAB_ATT_SRCS)

nps.CFLAGS += $(STAB_ATT_CFLAGS)
nps.srcs += $(STAB_ATT_SRCS)
