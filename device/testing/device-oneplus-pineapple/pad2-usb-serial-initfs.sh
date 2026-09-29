#!/bin/sh
set -eu

test -x /sysroot/usr/sbin/agetty
test -f /sysroot/usr/lib/systemd/system/pad2-usb-serial-rootfs.service

# Debug-shell cleanup has closed its gettys and removed ACM. Restore the
# function on the inherited network gadget; systemd will open the root shell.
cd /sys/kernel/config/usb_gadget/g1
udc=$(cat UDC)
test -n "$udc"
test ! -e functions/acm.usb0
test -L configs/c.1/ncm.usb0
printf '\n' > UDC

# Keep networking recoverable if restoring serial fails.
trap 'echo "USB serial setup failed; rebinding existing gadget" >&2; printf "%s\n" "$udc" > UDC' EXIT
mkdir functions/acm.usb0
test "$(cat functions/acm.usb0/port_num)" = 0
# configfs resolves the target at creation time from the caller's cwd.
ln -s /sys/kernel/config/usb_gadget/g1/functions/acm.usb0 configs/c.1/acm.usb0
trap - EXIT
printf '%s\n' "$udc" > UDC
echo 'OnePlus Pad 2: USB serial ready for rootfs debug shell'
