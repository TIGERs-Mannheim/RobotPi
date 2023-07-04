################################################################################
#
# robotpi
#
################################################################################

ROBOTPI_SITE = $(BR2_EXTERNAL_TIGERS_PATH)/package/robotpi
ROBOTPI_SITE_METHOD = local
ROBOTPI_DEPENDENCIES = systemd rpi-userland
ROBOTPI_OVERRIDE_SRCDIR_RSYNC_EXCLUSIONS = --include .git
ROBOTPI_SUPPORTS_IN_SOURCE_BUILD = NO

$(eval $(cmake-package))
