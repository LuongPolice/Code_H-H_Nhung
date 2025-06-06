BUZZER_VERSION = 1.0
BUZZER_SITE = $(TOPDIR)/package/buzzer
BUZZER_SITE_METHOD = local

define BUZZER_BUILD_CMDS
	$(MAKE) -C $(LINUX_DIR) M=$(BUZZER_SITE) modules
endef

define BUZZER_INSTALL_TARGET_CMDS
	$(MAKE) -C $(LINUX_DIR) M=$(BUZZER_SITE) INSTALL_MOD_PATH=$(TARGET_DIR) modules_install
endef

$(eval $(kernel-module))
$(eval $(generic-package))
