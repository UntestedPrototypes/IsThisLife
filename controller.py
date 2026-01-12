import socket
import pygame
import time
import select

# ================= CONFIG =================
ESP_ID = '1'        # ESP32 recipient ID
LAPTOP_ID = '0'     # Laptop ID
UDP_PORT = 4210
BROADCAST_IP = "255.255.255.255"
SEND_HZ = 50        # joystick send rate
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
print("Sending joystick to ESP32 ID =", ESP_ID)
print("Listening for messages to ID 0...\n")

last_send = 0.0

# ---------- Main loop ----------
while True:
    now = time.time()

    # ----- SEND joystick posXY -----
    if now - last_send >= SEND_PERIOD:
        pygame.event.pump()

        y = js.get_axis(0)   # left stick X
        x = js.get_axis(1)   # left stick Y

        payload = f"{x:.3f},{y:.3f}"
        msg = ESP_ID + payload   # <RID><PAYLOAD>
        # print(msg)

        sock.sendto(msg.encode(), (BROADCAST_IP, UDP_PORT))
        last_send = now

    # ----- RECEIVE ESP32 messages -----
    ready, _, _ = select.select([sock], [], [], 0)
    if ready:
        data, addr = sock.recvfrom(1024)
        text = data.decode(errors="ignore").strip()

        if not text:
            continue

        recipient = text[0]
        payload = text[1:]

        if recipient == LAPTOP_ID:
            print("ESP32:", payload)
