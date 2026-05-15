# Build cheatsheet

The canonical builds go through the Makefile:

```sh
make            # kernel module: vwifi_host.ko (needs kernel headers)
make userspace  # binaries: vwifi-medium, vwifi-host-relay, vwifi-phys-bridge
make test       # run the regression harness against vwifi-medium
make clean      # remove everything
```

Manual builds, if you can't use the Makefile:

```sh
gcc -Wall -Wextra -O2 -o vwifi-medium       vwifi_medium.c -lm
gcc -Wall -Wextra -O2 -o vwifi-host-relay   vwifi_host_relay.c
gcc -Wall -Wextra -O2 -o vwifi-phys-bridge  vwifi_phys_bridge.c
```

Userspace binaries don't link against any kernel headers, so they
build standalone in a CI environment without `linux-headers-*`.
