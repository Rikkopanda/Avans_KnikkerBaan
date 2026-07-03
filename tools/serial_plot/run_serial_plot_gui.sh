#!/bin/sh

# VS Code installed as a Snap can leak Snap runtime paths into child
# processes. Host-built Qt programs may then load /snap/core20 glibc pieces
# and fail with GLIBC_PRIVATE symbol lookup errors. Keep the display/session
# variables, but remove the Snap library and plugin paths.

unset LD_LIBRARY_PATH
unset LD_PRELOAD
unset SNAP
unset SNAP_ARCH
unset SNAP_COMMON
unset SNAP_CONTEXT
unset SNAP_COOKIE
unset SNAP_DATA
unset SNAP_EUID
unset SNAP_INSTANCE_NAME
unset SNAP_LIBRARY_PATH
unset SNAP_NAME
unset SNAP_REAL_HOME
unset SNAP_REVISION
unset SNAP_UID
unset SNAP_USER_COMMON
unset SNAP_USER_DATA
unset SNAP_VERSION
unset GTK_EXE_PREFIX
unset GTK_PATH
unset GDK_PIXBUF_MODULEDIR
unset GDK_PIXBUF_MODULE_FILE
unset GTK_IM_MODULE_FILE
unset GIO_MODULE_DIR
unset LOCPATH

# xcb matches the current app environment and avoids the harmless Wayland
# warning unless the caller explicitly asks for another Qt platform.
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
exec "$SCRIPT_DIR/serial_plot_gui" "$@"
