all: initramfs.cpio

initramfs.cpio: initramfs.conf
	$(REPO_DIR)/linux/usr/gen_init_cpio initramfs.conf > $@

.PHONY:clean

clean:
	rm initramfs.cpio
