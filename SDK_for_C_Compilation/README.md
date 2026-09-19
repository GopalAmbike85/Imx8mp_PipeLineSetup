# SDK for C compilation

Place the Toradex SDK installer here if you want the GitHub Actions review workflow to build automatically without relying on a repository secret.

Expected file name:

- tdx-xwayland-glibc-x86_64-cnr-image-ml-armv8a-verdin-imx8mp-toolchain-7.5.0.sh

The workflow checks this directory first before falling back to the TDX_TOOLCHAIN_URL secret.
