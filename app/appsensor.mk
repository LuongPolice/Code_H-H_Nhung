APPSENSOR_SITE = $(TOPDIR)/package/appsensor
APPSENSOR_SITE_METHOD = local
APPSENSOR_DEPENDENCIES = cjson

define APPSENSOR_BUILD_CMDS
	$(TARGET_CC) $(TARGET_CFLAGS) -o $(@D)/appsensor $(APPSENSOR_SITE)/app.c -lcjson
endef

define APPSENSOR_INSTALL_TARGET_CMDS
	$(INSTALL) -D -m 0755 $(@D)/appsensor $(TARGET_DIR)/usr/bin/appsensor
endef

$(eval $(generic-package))


