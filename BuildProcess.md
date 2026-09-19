# Build & Deploy Process

How to cross-compile `controller_bridge` (and the diagnostic tools under
`tools/`) for the Verdin i.MX8MP board, and how to get a built binary onto
the board and running. Written from the actual working steps used to build
and deploy this project - not aspirational instructions.

## 1. Prerequisites

All cross-compiling happens in **WSL2 Ubuntu** on the Windows dev machine
(`wsl -d Ubuntu`), not natively on Windows. The `F:\GA\channel-Robotics\`
tree is visible inside WSL at `/mnt/f/GA/channel-Robotics/`.

Required, already set up on the known-good dev machine:

- **Toradex SDK** at `/opt/tdx-xwayland/7.5.0` - provides the aarch64 cross
  toolchain, a sysroot with TensorFlow Lite 2.16.2 headers/libs
  (`tensorflow/lite/...`, `libtensorflow-lite.so`, `libvx_delegate.so` for
  the NPU), and `pkg-config` support.
- A **custom libwebsockets build with mbedTLS support**, at
  `~/tools/lws-aarch64-mbedtls/` (headers + `lib/pkgconfig/libwebsockets.pc`).
  The Toradex SDK's own libwebsockets is not used - this project needs one
  built with `LWS_WITH_MBEDTLS`.
- A matching **mbedTLS build** at `~/tools/mbedtls-aarch64/{include,lib}`.

If any of these are missing on a fresh machine, they need to be built first
(outside the scope of this doc - ask whoever set up the original dev
machine, or rebuild from the mbedTLS/libwebsockets source with
`-DLWS_WITH_MBEDTLS=ON` using the Toradex cross toolchain).

## 2. Cross-compiling `controller_bridge`

```bash
cd /mnt/f/GA/channel-Robotics/gitRepo/CNR-BaseStation-FW-Yocto/ControllerBridgeTflite
source /opt/tdx-xwayland/7.5.0/environment-setup-armv8a-tdx-linux

# Gotcha: the Toradex environment-setup script sets PKG_CONFIG_SYSROOT_DIR,
# which then makes pkg-config wrongly prepend the sysroot path onto our
# custom lws-aarch64-mbedtls .pc file (which lives OUTSIDE the sysroot,
# under $HOME). Unset it or the configure step fails to find libwebsockets.
unset PKG_CONFIG_SYSROOT_DIR
export PKG_CONFIG_PATH="$HOME/tools/lws-aarch64-mbedtls/lib/pkgconfig:$PKG_CONFIG_PATH"

# Sanity check BEFORE configuring - both of the failures below (wrong
# compiler cached, libwebsockets not found) trace back to running `cmake`
# before this block actually took effect in the current shell. If either
# of these is empty/missing, fix that first - don't proceed to `cmake` yet:
echo "CXX=$CXX"                                    # should NOT be empty
ls "$HOME/tools/lws-aarch64-mbedtls/lib/pkgconfig/libwebsockets.pc"  # should exist

mkdir -p build && cd build
cmake -S .. -B . \
  -DMBEDTLS_INCLUDE_DIR="$HOME/tools/mbedtls-aarch64/include" \
  -DMBEDTLS_LIB_DIR="$HOME/tools/mbedtls-aarch64/lib" \
  -DTFLITE_INCLUDE_DIR=/opt/tdx-xwayland/7.5.0/sysroots/armv8a-tdx-linux/usr/include \
  -DTFLITE_LIB_DIR=/opt/tdx-xwayland/7.5.0/sysroots/armv8a-tdx-linux/usr/lib

make -j"$(nproc)"
file controller_bridge   # sanity check: should read "ELF 64-bit LSB pie executable, ARM aarch64"
```

`CMakeLists.txt` prints a `WARNING` at configure time if
`TFLITE_INCLUDE_DIR`/`TFLITE_LIB_DIR` or `MBEDTLS_INCLUDE_DIR`/`MBEDTLS_LIB_DIR`
are left unset - don't ignore it, the build will fail later with a much less
obvious error (`mbedtls/ssl.h: No such file` or undefined `mbedtls_*`
reference errors) if you do.

### Gotcha: a bad `cmake` run poisons the build dir - `make` alone won't fix it

If `cmake` was run in a shell where `source environment-setup-armv8a-tdx-linux`
hadn't taken effect yet, it silently falls back to the **host's own**
`/usr/bin/c++` instead of the aarch64 cross-compiler, and CMake **caches**
that choice in `CMakeCache.txt`. Symptom: a build that mixes host C++
headers with the target sysroot's libc headers, failing with something like
`fatal error: bits/timesize-32.h: No such file or directory` (a glibc
time64-ABI header mismatch) deep inside `<libwebsockets.h>`'s own includes.

Once `CMAKE_CXX_COMPILER` is cached wrong, re-running `cmake`/`make` in that
**same** `build/` directory does NOT self-correct, even with the environment
now sourced correctly - CMake trusts the cache over a changed environment.
Confirm what's actually cached with:

```bash
grep CMAKE_CXX_COMPILER: build/CMakeCache.txt
# should read .../aarch64-tdx-linux/aarch64-tdx-linux-g++, NOT /usr/bin/c++
```

If it's wrong, `rm -rf build` and reconfigure from scratch (step 2's full
block above, in one shell where the sanity checks above both pass).

### Note on `$CC`

The environment-setup script's exported `$CC` variable has NOT reliably
expanded in every shell invocation used so far (came back empty in some
non-interactive `bash -c` calls). If a plain `$CC ...` invocation fails with
"command not found", call the cross-compiler by its full path instead:

```
/opt/tdx-xwayland/7.5.0/sysroots/x86_64-tdxsdk-linux/usr/bin/aarch64-tdx-linux/aarch64-tdx-linux-gcc
```

(same directory holds `aarch64-tdx-linux-g++` for the standalone tools under
`tools/`, if a C++ one is ever added).

## 3. Stripping the binary

`CROSS_COMPILE` is **not** set by `environment-setup-armv8a-tdx-linux`, so a
plain `strip controller_bridge` silently no-ops (wrong-architecture host
`strip` refuses/ignores an aarch64 ELF). Use the SDK's own cross-strip:

```bash
STRIP=/opt/tdx-xwayland/7.5.0/sysroots/x86_64-tdxsdk-linux/usr/bin/aarch64-tdx-linux/aarch64-tdx-linux-strip
cp controller_bridge controller_bridge_stripped
"$STRIP" controller_bridge_stripped
```

A correctly stripped build is ~67-68 KB; if `file` still shows `not
stripped` after this, the wrong strip binary was used.

## 4. Building the standalone tools (`tools/`)

Tools under `tools/<name>/` (currently just `tools/ntp_sniffer/`) are plain C,
no CMake, no TFLite/mbedTLS/libwebsockets dependency - just the cross
toolchain and standard Linux headers:

```bash
GCC=/opt/tdx-xwayland/7.5.0/sysroots/x86_64-tdxsdk-linux/usr/bin/aarch64-tdx-linux/aarch64-tdx-linux-gcc
SYSROOT=/opt/tdx-xwayland/7.5.0/sysroots/armv8a-tdx-linux
cd tools/ntp_sniffer
"$GCC" --sysroot="$SYSROOT" -Wall -Wextra -O2 -o ntp_sniffer ntp_sniffer.c
```

Strip the same way as step 3 if deploying it.

**Tip:** passing a multi-line build command through `wsl.exe` from a Windows
shell via inline `bash -c "..."` has been unreliable (quoting gets mangled,
variables come back empty). Write the build steps to a `.sh` file and run
`wsl -d Ubuntu -- bash /mnt/c/.../that_script.sh` instead. If invoking from
Git Bash on Windows, prefix with `MSYS_NO_PATHCONV=1` so it doesn't mangle
`/mnt/...` paths into bogus Windows paths.

## 5. Deploying to the board

The board (Verdin i.MX8MP, hostname `verdin-imx8mp`) has **no SSH server** -
deployment is serial console (for shell access) + TFTP (for file transfer).

### 5.1 Serial console

- 115200 8N1, already logged in as `root`.
- COM port **varies between sessions** (seen as COM4 and COM6 on different
  days) - always verify which port is live before assuming.
- Bulk `SerialPort.WriteLine()` can corrupt characters under load (e.g. a
  space silently inserted mid-filename). Send character-by-character with a
  short delay, and always verify what was actually received via
  `/proc/<pid>/cmdline` on the board - never trust the local terminal echo
  alone. It can look corrupted when the send was actually fine, or look
  fine when it wasn't.

### 5.2 File transfer (TFTP)

The board's busybox image has `tftp` (client) but no `ssh`/`scp` server.
Run a TFTP server **on the Windows host** (not WSL - WSL2's own IP is not
reliably reachable from the board's LAN due to NAT), bound to whichever
adapter shares the board's subnet:

1. Find the board's IP/subnet: `ip addr show end0` on the board.
2. Find the matching Windows adapter: `ipconfig` - look for an adapter on
   that same subnet. **This has changed between sessions** (seen on both the
   Ethernet and WiFi adapters, and the board's own subnet has changed too,
   e.g. `192.168.40.x` vs `192.168.0.x`) - always re-verify both sides, don't
   assume a prior session's IPs still apply.
3. Start a TFTP server on the Windows host bound to that adapter's IP, port
   69, serving a staging directory with the file(s) to push.
4. From the board's serial console: `tftp -g -r <filename> -l <filename>
   <windows-host-ip>` to pull a file onto the board.

For multiple files (e.g. a full redeploy after the board's `~/tls_deploy/`
directory gets wiped - see 5.4), bundle everything into one `tar.gz` first
and push that single archive, then `tar xzf` it on the board - far fewer
serial commands than transferring each file individually, and fewer chances
for a corrupted filename mid-command.

### 5.2a Assembling the `tls_deploy` package

Everything `/root/tls_deploy/` on the board needs, and where each piece
actually comes from (verified by `cmp`, not assumed):

| File | Origin |
|---|---|
| `controller_bridge` | This project's own build output (steps 2-3 above), stripped |
| `libmbedcrypto.so.16`, `libmbedtls.so.21`, `libmbedx509.so.7` | Tracked in this repo under `deploy/` - exact copies of the same aarch64 mbedTLS build at `~/tools/mbedtls-aarch64/lib/` used to link the binary in step 2 |
| `libwebsockets.so.19` | Tracked in this repo under `deploy/` - exact copy of `~/tools/lws-aarch64-mbedtls/lib/libwebsockets.so.19`, used to link the binary in step 2 |
| `pc_server.crt`, `pc_server.key` | Tracked in this repo under `deploy/` - pre-existing self-signed **test** certificate (`CN=fw-update-server`, issued by `Pinetics Test Root CA`, valid 2026-08-17 to 2036-08-14). Not a production secret, committed deliberately for reproducibility - replace with a real cert/key (not committed) before any non-test deployment. |
| `TfLSTM_l2_w128_s20_int8.tflite` | This project's own `models/` directory |

Only `controller_bridge` itself needs building per-deploy (steps 2-3); the
rest already live in this repo under `deploy/`, so assembling the package is
just a copy, not a rebuild:

```bash
# --- On the WSL/Windows side: assemble the package ---
mkdir -p deploy_pkg
cp build/controller_bridge_stripped deploy_pkg/controller_bridge
cp deploy/*.so.* deploy/pc_server.crt deploy/pc_server.key deploy_pkg/
cp models/TfLSTM_l2_w128_s20_int8.tflite deploy_pkg/
chmod +x deploy_pkg/controller_bridge deploy_pkg/pc_server.crt deploy_pkg/pc_server.key
(cd deploy_pkg && tar czf ../tls_deploy.tar.gz .)

# Start a TFTP server on the Windows host (see 5.2 above), bound to the
# adapter on the board's subnet, serving the directory holding
# tls_deploy.tar.gz.

# --- On the board's serial console: pull and extract ---
mkdir -p /root/tls_deploy && cd /root/tls_deploy
tftp -g -r tls_deploy.tar.gz -l tls_deploy.tar.gz <windows-host-ip>
tar xzf tls_deploy.tar.gz && rm tls_deploy.tar.gz

LD_LIBRARY_PATH=. nohup ./controller_bridge 8081 8080 pc_server.crt pc_server.key \
    TfLSTM_l2_w128_s20_int8.tflite /usr/lib/libvx_delegate.so timing_log.csv \
    > controller_bridge.log 2>&1 &
```

### 5.3 Running the binary - `LD_LIBRARY_PATH`

The cross-compiled binary's `RUNPATH` points at the WSL build machine's own
library path, which doesn't exist on the board. Launch it with
`LD_LIBRARY_PATH=.` from the directory holding the matching shared libs:

```bash
cd /root/tls_deploy
LD_LIBRARY_PATH=. nohup ./controller_bridge 8081 8080 pc_server.crt pc_server.key \
    TfLSTM_l2_w128_s20_int8.tflite /usr/lib/libvx_delegate.so timing_log.csv \
    > controller_bridge.log 2>&1 &
```

Argument order: `controller_port icu_port cert_path key_path model_path
delegate_path timing_log_path`. Pass `cpu` (literal) as `delegate_path` to
force the CPU fallback instead of the NPU. `timing_log_path` is optional -
omit it (or pass `""`) to disable per-frame CSV logging.

`/root/tls_deploy/` needs, alongside the binary: `pc_server.crt`,
`pc_server.key`, the `.tflite` model, and
`libmbedcrypto.so.16`/`libmbedtls.so.21`/`libmbedx509.so.7`/`libwebsockets.so.19`
(only `libwebsockets.so.19` is in the board's system `ldconfig` cache -
mbedTLS is not, hence `LD_LIBRARY_PATH=.`).

### 5.4 Known board quirk - `~/tls_deploy/` can vanish

This directory has been observed **completely wiped** both mid-session with
no reboot in between (cause unspecified, confirmed as known/expected board
behavior) **and** confirmed on every reboot (verified directly: after a
reboot with ~2h14m uptime, `/root/tls_deploy/` was simply gone, `ls` reported
"No such file or directory") - `/root/` is very likely on a non-persistent
tmpfs/overlay rather than the board's real persistent storage. Don't assume
a previous session's deployed files are still present after ANY gap,
reboot or not - check with `ls` before assuming, and keep a ready-to-push
staging copy (binary + all four shared libs + cert/key, see 5.2a) so a full
redeploy is one `tar.gz` + `tftp -g` + `tar xzf` away rather than a
from-scratch rebuild.

Also worth checking after any reboot: the board's system clock (`date`).
It has been observed both correct and wildly wrong (stuck over a year in the
past) across different sessions/reboots on this same board - don't assume
timestamp-based fields (`frame_transfer_time` etc.) are meaningful without
checking `date` first.

### 5.5 Never overwrite a file a running process has open

This applies to the binary itself **and** any shared file a running process
has `mmap()`'d - the `.tflite` model included. `FlatBufferModel::BuildFromFile()`
mmaps the model file; overwriting it (even via a same-named TFTP push) while
the old process still has it mapped has caused a live `SIGBUS` crash before.

Always stage a new binary/model under a **new filename**, confirm it landed
correctly (size check, ideally `/proc/<pid>/cmdline` after starting it), kill
the old process, then start the new one - never overwrite in place while
something might still be using the old file.

### 5.6 Sanity checks after (re)starting

- `ps | grep controller_bridge` - confirm the new PID is up.
- `tr '\0' '|' < /proc/<pid>/cmdline; echo` - confirm the **actual** argv the
  process received (don't trust the serial echo).
- `tail -f <logfile>` - confirm both vhosts came up, NPU warm-up completed
  (~10s, only on first start with the NPU delegate), and the controller
  reconnects on its own (normally within a couple of seconds).

### 5.7 Auto-starting on boot (systemd)

The board runs **systemd** (confirmed: `/etc/systemd/system/` is populated,
no `/etc/init.d`). `systemd/controller-bridge.service` in this repo is a
verified-working unit (tested with two real power-cycles of the actual
board, not just enabled-and-assumed) that starts `controller_bridge` from
`/root/tls_deploy/` automatically on boot.

```bash
# Push the unit file into place (same TFTP approach as 5.2), then:
systemctl daemon-reload
systemctl enable controller-bridge.service
# Optional - start it immediately instead of waiting for the next reboot:
systemctl start controller-bridge.service
systemctl status controller-bridge.service
```

**Gotcha found and fixed by testing**: the unit originally used
`After=network-online.target` / `Wants=network-online.target` (the "textbook"
way to wait for network before starting a service). On this board,
`network-online.target` **never activates** - there's no
`systemd-networkd-wait-online`/`NetworkManager-wait-online` wired in - so a
`Wants=`/`After=` dependency on it blocks the service **indefinitely**, not
just orders it later. Confirmed via `systemctl status
controller-bridge.service` showing `enabled` but stuck `inactive (dead)`
minutes after a real boot, while `systemctl status network-online.target`
itself also showed `inactive (dead)`. Fixed by ordering after
`end0-network.service` instead (the actual Toradex unit that brings up the
interface and runs DHCP) - `After=` only, no `Wants=`/`Requires=`, so a
slow or failed DHCP doesn't block the app from starting (it tolerates
starting before the network is fully configured; it just won't accept real
connections until DHCP completes).

Verified end-to-end on real hardware, twice: `reboot` from the serial
console → board power-cycles (`Reset cause: POR` in the U-Boot log, a real
cold reset, not a soft restart) → `controller-bridge.service` shows
`active (running)` within ~30s of boot, NPU warm-up included, with no manual
`systemctl start` needed.

**Still true / unresolved**: exactly why `/root/tls_deploy/` sometimes
disappears (see 5.4) remains unconfirmed - across the two reboots used to
verify this service, one preceded a wipe and one didn't, so the trigger is
NOT simply "every reboot". If `tls_deploy/` is missing when the board boots,
this service will be `enabled` but fail to start (`ExecStart` file not
found) - check `systemctl status controller-bridge.service` after any
unexpected outage and redeploy per 5.2a if so.

## 6. Diagnostic tools reference

- `tools/ntp_sniffer/` - standalone raw-socket tool (not part of
  `controller_bridge`) that watches real NTP (UDP port 123) traffic on an
  interface and decodes request/response pairs. Needed because this board
  has no `tcpdump`/`libpcap`. Must bind with `ETH_P_ALL`, not `ETH_P_IP` -
  the kernel only mirrors locally-generated (outgoing) packets to
  `ETH_P_ALL` listeners, so `ETH_P_IP` only ever sees inbound traffic.
  Usage: `./ntp_sniffer [interface=end0] [filter_ip]`.
