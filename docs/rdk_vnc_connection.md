# RDK Remote Desktop Access Using x11vnc

## Overview

This document describes how to configure an RDK board to share its graphical desktop using `x11vnc` and how to connect to it from a remote computer using any VNC client. Remmina is used only as an example client.

The connection is made directly over the local network and does not require Internet access.

---

# Prerequisites

- An RDK board running Linux with a graphical desktop (X11).
- `x11vnc` installed on the RDK.
- The RDK and the client computer connected to the same network (Ethernet or Wi-Fi).
- A VNC client installed on the client computer.

---

# Step 1: Start the VNC Server on the RDK

Open a terminal on the RDK and start `x11vnc`:

```bash
x11vnc -display :0 -forever -shared -nopw
```

## Option Description

| Option | Description |
|----------|-------------|
| `-display :0` | Shares the main graphical display. |
| `-forever` | Keeps the server running after a client disconnects. |
| `-shared` | Allows multiple clients to connect simultaneously. |
| `-nopw` | Disables password authentication. Suitable only for trusted local networks. |

If access to the display is denied, try:

```bash
sudo x11vnc -auth guess -display :0 -forever -shared -nopw
```

---

# Step 2: Determine the RDK IP Address

On the RDK:

```bash
hostname -I
```

Example output:

```text
192.168.1.42
```

This IP address will be used by the VNC client.

---

# Step 3: Verify That the Server Is Listening

Check that `x11vnc` is listening on the default VNC port:

```bash
ss -tlnp | grep 5900
```

Expected result:

```text
LISTEN 0 32 0.0.0.0:5900
```

If no output is shown, the VNC server is not running correctly.

---

# Step 4: Verify Network Connectivity

From the client computer, test communication with the RDK:

```bash
ping 192.168.1.42
```

Replace the IP address with the actual address of the RDK.

Successful replies confirm that both devices can communicate over the network.

---

# Step 5: Connect Using a VNC Client

Any VNC client may be used.

General connection parameters:

| Setting | Value |
|-----------|---------|
| Protocol | VNC |
| Server Address | `<RDK_IP>` |
| Port | `5900` |

Example:

```text
192.168.1.42:5900
```

---

# Example: Connecting with Remmina

1. Launch Remmina.
2. Create a new connection profile.
3. Select **VNC** as the protocol.
4. Enter the RDK IP address:
   ```
   192.168.1.42
   ```
5. Save the profile.
6. Click **Connect**.

The RDK desktop should appear in the Remmina window.

---

# Configuring Automatic Startup

To start the VNC server automatically after boot, create a systemd service.

Create:

```bash
sudo nano /etc/systemd/system/x11vnc.service
```

Insert:

```ini
[Unit]
Description=Start x11vnc at startup
After=graphical.target
Wants=graphical.target

[Service]
Type=simple
ExecStartPre=/bin/sleep 10
ExecStart=/usr/bin/x11vnc -auth guess -display :0 -forever -shared -nopw
Restart=always
RestartSec=5

[Install]
WantedBy=graphical.target
```

Enable the service:

```bash
sudo systemctl daemon-reload
sudo systemctl enable x11vnc.service
sudo systemctl start x11vnc.service
```

Verify:

```bash
systemctl status x11vnc.service
```

The service should report:

```text
active (running)
```

---

# Troubleshooting

## Unable to Connect

Verify that the VNC server is running:

```bash
ps aux | grep x11vnc
```

Verify that port 5900 is open:

```bash
ss -tlnp | grep 5900
```

Verify network communication:

```bash
ping <RDK_IP>
```

---

## No Display Available

Check whether the graphical display exists:

```bash
ls /tmp/.X11-unix/
```

A running desktop session typically shows:

```text
X0
```

If no display exists, the system may be running headless and no graphical session is available for `x11vnc` to share.

---

## Firewall Issues

Ensure that TCP port 5900 is not blocked by a firewall on the RDK.

---

# Network Requirements

Internet access is not required.

The only requirement is that:

- The RDK and the client computer are on the same network.
- The client can reach the RDK IP address.

Typical example:

```text
Client PC: 192.168.1.10
RDK:       192.168.1.42
```

The VNC client communicates directly with the RDK over the local network.