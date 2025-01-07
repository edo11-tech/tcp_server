#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
import math
from sensor_msgs.msg import LaserScan
import json

class TcpLaserScanNode(Node):

    def __init__(self):
        super().__init__('tcp_laserscan_node')  # Nome del nodo

        # Configura il server TCP
        self.HOST = '0.0.0.0'
        self.PORT = 5169
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Parametri del LaserScan
        self.angle_min = 0.0  # Angolo minimo [rad]
        self.angle_max = 2 * math.pi  # Angolo massimo [rad]
        self.angle_increment = 0.0174533  # Incremento angolare [rad] (1 grado)
        self.scan_time = 0.1  # Tempo tra scansioni [s]
        self.range_min = 0.1  # Range minimo [m]
        self.range_max = 10.0  # Range massimo [m]

        # Publisher per i dati LaserScan
        self.laserscan_publisher = self.create_publisher(LaserScan, 'scan', 10)

        # Sottoscrittore per inviare dati al client TCP
        self.subscription = self.create_subscription(
            LaserScan,
            'tcp_send_scan',
            self.send_to_client_callback,
            10
        )
        self.subscription  # Impedisce la garbage collection

        self.client_socket = None
        self.client_address = None

        # Avvia il server TCP
        self.start_tcp_server()

    def start_tcp_server(self):
        try:
            # Configura e avvia il server TCP
            self.server_socket.bind((self.HOST, self.PORT))
            self.server_socket.listen(1)
            self.get_logger().info(f"Server TCP in ascolto su {self.HOST}:{self.PORT}...")

            # Accetta connessioni
            self.client_socket, self.client_address = self.server_socket.accept()
            self.get_logger().info(f"Connessione stabilita con {self.client_address}")

            # Avvia il loop di ricezione
            self.create_timer(0.1, self.receive_data)

        except Exception as e:
            self.get_logger().error(f"Errore durante l'avvio del server TCP: {e}")
            self.destroy_node()



    def receive_data(self):
        if self.client_socket:
            try:
                # Riceve dati dal client
                data = self.client_socket.recv(4096)
                if data:
                    message = data.decode('utf-8').strip()
                    self.get_logger().info(f"Messaggio ricevuto: {message}")
                    
                    try:
                        # Parso il messaggio come JSON
                        message_json = json.loads(message)

                        # Controllo se le chiavi 'angle' e 'distance' esistono
                        if 'angle' in message_json and 'distance' in message_json:
                            angles = message_json['angle']
                            distances = message_json['distance']

                            # Verifica che siano liste della stessa lunghezza
                            if isinstance(angles, list) and isinstance(distances, list) and len(angles) == len(distances):
                                # Costruisce il messaggio LaserScan
                                scan_msg = LaserScan()
                                scan_msg.header.stamp = self.get_clock().now().to_msg()
                                scan_msg.header.frame_id = "laser_frame"
                                scan_msg.angle_min = float(min(angles)) * (3.14159 / 180.0)
                                scan_msg.angle_max = float(max(angles)) * (3.14159 / 180.0)
                                scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / len(angles)
                                scan_msg.time_increment = 0.0
                                scan_msg.scan_time = 0.0
                                scan_msg.range_min = 0.1
                                scan_msg.range_max = 10.0
                                scan_msg.ranges = distances
                                scan_msg.intensities = []

                                # Pubblica il messaggio
                                self.laserscan_publisher.publish(scan_msg)
                                self.get_logger().info(f"LaserScan pubblicato con {len(angles)} angoli.")
                            else:
                                self.get_logger().error("Le liste 'angle' e 'distance' non hanno la stessa lunghezza o non sono valide.")
                        else:
                            self.get_logger().error("Chiavi 'angle' o 'distance' mancanti nel messaggio JSON.")
                    except json.JSONDecodeError as e:
                        self.get_logger().error(f"Errore durante il parsing del messaggio JSON: {e}")
                else:
                    self.get_logger().info("Connessione chiusa dal client.")
                    self.client_socket.close()
                    self.client_socket = None

            except Exception as e:
                self.get_logger().error(f"Errore durante la ricezione dei dati: {e}")
    

    def parse_lidar_data(self, message):
        """
        Converte il messaggio TCP in un array di distanze.
        Supponiamo che i dati siano inviati come stringa CSV: "0.5,1.0,1.5,..."
        """
        try:
            ranges = [float(x) for x in message.split(',')]
            return ranges
        except ValueError:
            self.get_logger().error("Formato dei dati non valido.")
            return None

    def publish_laser_scan(self, ranges):
        """
        Pubblica i dati ricevuti nel formato `sensor_msgs/msg/LaserScan`.
        """
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'  # Cambia se necessario
        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_max
        msg.angle_increment = self.angle_increment
        msg.time_increment = self.scan_time / len(ranges)
        msg.scan_time = self.scan_time
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        msg.ranges = ranges

        self.laser_publisher.publish(msg)
        self.get_logger().info("Messaggio LaserScan pubblicato.")

    def send_to_client_callback(self, msg):
        if self.client_socket:
            try:
                # Invia il messaggio al client
                serialized_data = ','.join(map(str, msg.ranges))
                self.client_socket.sendall(serialized_data.encode('utf-8'))
                self.get_logger().info("Messaggio LaserScan inviato al client.")
            except Exception as e:
                self.get_logger().error(f"Errore durante l'invio dei dati al client: {e}")
        else:
            self.get_logger().warn("Nessun client connesso. Impossibile inviare il messaggio.")

    def destroy_node(self):
        # Chiudi il socket quando il nodo viene distrutto
        if self.client_socket:
            self.client_socket.close()
        self.server_socket.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TcpLaserScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
