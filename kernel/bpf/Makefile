obj-y := core.o
CFLAGS_core.o += $(call cc-disable-warning, override-init)

obj-$(CONFIG_BPF_SYSCALL) += syscall.o verifier.o
ifdef CONFIG_TEST_BPF
obj-$(CONFIG_BPF_SYSCALL) += test_stub.o
endif
