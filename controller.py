import socket
import pygame
import time
import select

# ================= CONFIG =================
ESP_ID = '1'
LAPTOP_ID = '0'
UDP_PORT = 4210
BROADCAST_IP = "255.255.255.255"
SEND_HZ = 50
DEADZONE = 0.15
# =========================================

SEND_PERIOD = 1.0 / SEND_HZ

# ---------- UDP socket ----------
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
sock.bind(("", UDP_PORT))
sock.setblocking(False)

# ---------- Controller ----------
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    raise RuntimeError("No controller detected")

js = pygame.joystick.Joystick(0)
js.init()

print("Controller detected:", js.get_name())

last_send = 0.0

# ---------- Main loop ----------
while True:
    now = time.time()

    if now - last_send >= SEND_PERIOD:
        pygame.event.pump()

        y = js.get_axis(0)
        x = js.get_axis(1)

        # Deadzone gate
        if abs(x) > DEADZONE or abs(y) > DEADZONE:
            sx = -x
            sy = -y
        else:
            sx = 0.0
            sy = 0.0

        payload = f"{sx:.3f},{sy:.3f}"
        msg = ESP_ID + payload
        sock.sendto(msg.encode(), (BROADCAST_IP, UDP_PORT))
        last_send = now

    ready, _, _ = select.select([sock], [], [], 0)
    if ready:
        data, addr = sock.recvfrom(1024)
        text = data.decode(errors="ignore").strip()

        if not text:
            continue

        if text[0] == LAPTOP_ID:
            print("ESP32:", text[1:])
