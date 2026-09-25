#!/usr/bin/python3
"""
Headless TCP entry point (legacy app on ports 5000/8000), as a systemd-style
service with signal control.

With the multiprocess split, ``Server()`` forks P_control / P_sensors / P_camera /
P_aux and its own Supervisor watches them, so the job of this file shrank: it no
longer needs its own restart-on-crash wrapper around three threads. What is left
is the TCP transports (which are threads in P_web, because they are pure socket
I/O) and the signal plumbing.

One thing that MUST NOT come back here: ``os._exit`` or a hard kill before
``Server.shutdown()``. The PCA9685 latches its last duty in hardware, so leaving
the children orphaned leaves the kart driving.
"""
import logging
import os
import signal
import sys
import threading
import time

from server import Server

# Configuration log
logging.basicConfig(filename='/var/log/car_server.log', level=logging.INFO,
                    format='%(asctime)s - %(levelname)s - %(message)s')


class ServerController:
    def __init__(self):
        # Forks the child processes; must happen before any thread starts here.
        self.TCP_Server = Server()
        self.is_running = False
        self.threads = []
        self.stop_event = threading.Event()

    def beep(self):
        """Startup chirp, through P_aux (which owns the buzzer pin)."""
        try:
            self.TCP_Server._to_aux('buzzer', on='1')
            time.sleep(0.5)
            self.TCP_Server._to_aux('buzzer', on='0')
        except Exception as e:
            logging.error(f"Buzzer error: {e}")

    def start_server(self):
        if self.is_running:
            logging.info("Server is already running")
            return

        logging.info("Starting server...")
        self.TCP_Server.tcp_Flag = True
        self.TCP_Server.StartTcpServer()
        # These three are socket/queue I/O only -- no hardware, no busy loops --
        # so they stay threads in this process. Each loop exits on its own when
        # tcp_Flag drops and its socket closes, which is why the old
        # restart-on-exception wrapper is gone: an exiting loop now means "we are
        # shutting down", not "it crashed, restart it".
        self.threads = [
            threading.Thread(target=self.TCP_Server.readdata, name="ReadData",
                             daemon=True),
            threading.Thread(target=self.TCP_Server.sendvideo, name="SendVideo",
                             daemon=True),
            threading.Thread(target=self.TCP_Server.Power, name="Power",
                             daemon=True),
        ]
        for thread in self.threads:
            thread.start()
        self.is_running = True
        logging.info("Server started: %s", self.TCP_Server.supervisor.status())

        threading.Thread(target=self.beep, daemon=True).start()

    def stop_server(self):
        if not self.is_running:
            logging.info("Server is not running")
            return
        logging.info("Stopping server...")
        self.TCP_Server.StopTcpServer()          # drops tcp_Flag, closes sockets
        for thread in self.threads:
            thread.join(timeout=3)
        self.is_running = False
        logging.info("Server stopped")

    def run(self):
        self.start_server()
        try:
            while not self.stop_event.is_set():
                time.sleep(1)
                # The Supervisor logs and acts on a dead or wedged child itself
                # (including braking the motors); surface it in the service log
                # too, since that is what an operator reads after the fact.
                degraded = self.TCP_Server.supervisor.degraded
                if degraded:
                    logging.error("degraded processes: %s", degraded)
        except KeyboardInterrupt:
            logging.info("Program interrupted by user")
        finally:
            self.stop_server()

    def shutdown(self):
        self.stop_event.set()
        self.stop_server()
        self.TCP_Server.shutdown()               # joins children, brakes motors


def cleanup(controller):
    logging.info("Cleaning up resources...")
    try:
        controller.shutdown()
    except Exception as e:
        logging.error(f"Error during shutdown: {e}")
    try:
        import RPi.GPIO as GPIO
        GPIO.cleanup()
    except Exception as e:
        logging.error(f"Error during GPIO cleanup: {e}")


if __name__ == '__main__':
    controller = ServerController()

    def handle_stop(signum, frame):
        logging.info("Stop signal received")
        controller.stop_server()

    def handle_restart(signum, frame):
        logging.info("Restart signal received")
        controller.stop_server()
        controller.start_server()

    def handle_shutdown(signum, frame):
        logging.info("Shutdown signal received")
        controller.stop_event.set()

    # SIGINT/SIGTERM only SET the stop flag; the actual teardown runs in the main
    # thread's finally block. Doing it inside the handler risked re-entering
    # shutdown from a signal while the main thread was already in it.
    signal.signal(signal.SIGINT, handle_shutdown)
    signal.signal(signal.SIGTERM, handle_shutdown)
    signal.signal(signal.SIGUSR1, handle_stop)      # stop the TCP transports
    signal.signal(signal.SIGUSR2, handle_restart)   # restart them

    try:
        controller.run()
    finally:
        cleanup(controller)
        logging.info("Exiting")
        sys.exit(0)
