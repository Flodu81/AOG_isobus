#!/usr/bin/env python3
"""
ota_send.py - envoie un firmware .hex a un Teensy 4.1 par UDP (port 8888, PGN 0xB0).

Usage :  python ota_send.py 192.168.1.126 firmware.hex
         python ota_send.py 192.168.1.126 firmware.hex --port 8888 --chunk 1024
Double-clic : le script demande l'adresse IP et le fichier .hex, puis attend
              une touche avant de fermer la fenetre.

Remarque : fermer AgIO pendant la mise a jour (il envoie aussi sur le port 8888).
"""
import argparse
import socket
import struct
import sys
import time

PGN_OTA = 0xB0


def frame(seq: int, data: bytes = b"") -> bytes:
    body = bytes([0x80, 0x81, 0x7F, PGN_OTA]) + struct.pack("<H", seq) + data
    return body + bytes([sum(body[2:]) & 0xFF])


def wait_ack(sock: socket.socket, seq: int, timeout: float):
    """Retourne le statut du Teensy pour ce seq, ou None si timeout."""
    end = time.time() + timeout
    while True:
        left = end - time.time()
        if left <= 0:
            return None
        sock.settimeout(left)
        try:
            d, _ = sock.recvfrom(64)
        except socket.timeout:
            return None
        if (len(d) == 8 and d[:4] == bytes([0x80, 0x81, 0x7E, PGN_OTA])
                and (sum(d[2:7]) & 0xFF) == d[7]
                and (d[4] | (d[5] << 8)) == seq):
            return d[6]


def send(sock, dst, seq, data=b"", timeout=1.0, retries=10) -> int:
    pkt = frame(seq, data)
    for _ in range(retries):
        sock.sendto(pkt, dst)
        status = wait_ack(sock, seq, timeout)
        if status is not None:
            return status
    raise RuntimeError(f"pas de reponse du Teensy (paquet {seq})")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("ip", nargs="?")
    ap.add_argument("hexfile", nargs="?")
    ap.add_argument("--port", type=int, default=8888)
    ap.add_argument("--chunk", type=int, default=1024, help="octets par paquet (<= 1400)")
    args = ap.parse_args()

    if not args.ip:
        args.ip = input("Adresse IP du Teensy (ex. 192.168.1.126) : ").strip()
    if not args.hexfile:
        args.hexfile = input("Fichier .hex (glisse-le ici puis Entree) : ").strip().strip('"')

    try:
        with open(args.hexfile, "rb") as f:
            data = f.read()
    except OSError as e:
        print("Impossible d'ouvrir le fichier :", e)
        return 1
    if not data.startswith(b":"):
        print("Ce fichier ne ressemble pas a un .hex Intel.")
        return 1
    if not data.endswith(b"\n"):
        data += b"\n"

    dst = (args.ip, args.port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", 0))          # port source libre : le Teensy repond a l'emetteur

    print("Demarrage de la mise a jour...")
    try:
        # Le Teensy prepare la flash avant de repondre : on laisse jusqu'a ~30 s.
        status = send(sock, dst, 0, timeout=5.0, retries=6)
    except RuntimeError as e:
        print("Echec :", e)
        return 1
    if status == 1:
        print("Refuse : le guidage est engage. Desengage puis recommence.")
        return 1
    if status == 2:
        print("Refuse : pas de zone flash libre pour recevoir le firmware.")
        return 1

    chunks = [data[i:i + args.chunk] for i in range(0, len(data), args.chunk)]
    t0 = time.time()
    try:
        for n, c in enumerate(chunks, start=1):
            send(sock, dst, n, c)
            if n % 20 == 0 or n == len(chunks):
                print(f"\r{n}/{len(chunks)} paquets", end="", flush=True)
    except RuntimeError as e:
        print("\nEchec :", e)
        print("Le Teensy redemarrera tout seul sur l'ancien firmware (silence > 8 s).")
        return 1

    print(f"\nEnvoi termine en {time.time() - t0:.1f} s. "
          "Le Teensy verifie, flashe et redemarre (quelques secondes).")
    return 0


if __name__ == "__main__":
    code = 1
    try:
        code = main()
    except Exception as e:      # affiche l'erreur au lieu de fermer la fenetre
        print("Erreur :", e)
    if len(sys.argv) < 3:       # lance par double-clic : on laisse la fenetre ouverte
        input("\nAppuie sur Entree pour fermer...")
    sys.exit(code)
