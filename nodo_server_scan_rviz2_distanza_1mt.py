#!/usr/bin/env python3
# Specifica che il file deve essere eseguito con l'interprete Python 3.

import rclpy
from rclpy.node import Node
# Importa la libreria di ROS 2 rclpy e il modulo Node per creare un nodo ROS.

import socket
# Importa la libreria socket per la comunicazione di rete TCP.

import math
# Importa il modulo math per funzioni matematiche come pi.

from sensor_msgs.msg import LaserScan
# Importa il tipo di messaggio LaserScan dalla libreria sensor_msgs, che è usato per pubblicare i dati di scansione del laser.

import json
# Importa il modulo json per gestire i dati in formato JSON.

class TcpLaserScanNode(Node):
    # Definisce una classe che estende Node di ROS 2 per creare un nodo che gestisce una comunicazione TCP.
    def __init__(self):
        super().__init__('tcp_laserscan_node')  # Inizializza il nodo con il nome 'tcp_laserscan_node'.
        
        # Configura il server TCP
        self.HOST = '0.0.0.0'  # Imposta l'indirizzo IP del server, 0.0.0.0 accetta connessioni da tutte le interfacce.
        self.PORT = 5169  # Imposta la porta TCP su cui il server ascolterà.
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Crea un socket TCP (IPv4).

        # Parametri del LaserScan
        self.angle_min = 0.0  # Angolo minimo per la scansione laser (in radianti).
        self.angle_max = 2 * math.pi  # Angolo massimo per la scansione laser (in radianti).
        self.angle_increment = 0.0174533  # Incremento angolare per la scansione laser (1 grado in radianti).
        self.scan_time = 0.1  # Tempo tra le scansioni (in secondi).
        self.range_min = 0.1  # Distanza minima rilevabile dal laser (in metri).
        self.range_max = 10.0  # Distanza massima rilevabile dal laser (in metri).

        # Publisher per i dati LaserScan
        self.laserscan_publisher = self.create_publisher(LaserScan, 'scan', 10)
        # Crea un publisher per pubblicare messaggi di tipo LaserScan sul topic 'scan' con una coda di 10 messaggi.

        # Sottoscrittore per inviare dati al client TCP
        self.subscription = self.create_subscription(
            LaserScan,
            'tcp_send_scan',
            self.send_to_client_callback,
            10
        )
        self.subscription  # Impedisce la garbage collection.

        self.client_socket = None  # Inizializza il socket del client a None.
        self.client_address = None  # Inizializza l'indirizzo del client a None.
        
        # Aggiungi il buffer per accumulare i dati ricevuti
        self.buffer = ""  # Inizializza un buffer vuoto

        # Avvia il server TCP
        self.start_tcp_server()

    def start_tcp_server(self):
        try:
            # Configura e avvia il server TCP
            self.server_socket.bind((self.HOST, self.PORT))  # Associa il socket a un indirizzo e porta.
            self.server_socket.listen(1)  # Imposta il server in modalità di ascolto, massimo una connessione in attesa.
            self.get_logger().info(f"Server TCP in ascolto su {self.HOST}:{self.PORT}...")
            # Logga un messaggio che indica che il server sta ascoltando.

            # Accetta connessioni
            self.client_socket, self.client_address = self.server_socket.accept()
            # Accetta una connessione in ingresso dal client.
            self.get_logger().info(f"Connessione stabilita con {self.client_address}.")
            # Logga il messaggio di connessione stabilita.

            # Avvia il loop di ricezione
            self.create_timer(0.1, self.receive_data)  # Crea un timer che chiama il metodo 'receive_data' ogni 0.1 secondi.

        except Exception as e:
            self.get_logger().error(f"Errore durante l'avvio del server TCP: {e}")
            self.destroy_node()
            # Gestisce l'errore in caso di fallimento nel setup del server TCP, distruggendo il nodo.

    def receive_data(self):
        if self.client_socket:
            try:
                # Riceve dati dal client
                data = self.client_socket.recv(4096)  # Legge fino a 4096 byte di dati dal client.
                if data:
                    # Decodifica i dati ricevuti e accumulali in un buffer
                    self.buffer += data.decode('utf-8')
                    
                    # Processa ogni messaggio JSON completo (delimitato da \n)
                    while '\n' in self.buffer:
                        message, self.buffer = self.buffer.split('\n', 1)
                        self.get_logger().info(f"Messaggio ricevuto: {message}")

                        # Parso il messaggio come JSON
                        try:
                            message_json = json.loads(message)

                            # Controllo se le chiavi 'angle' e 'distance' esistono
                            if 'angle' in message_json and 'distance' in message_json:
                                angle = message_json['angle']  # Ottiene l'angolo.
                                distance = message_json['distance']  # Ottiene la distanza.

                                # Costruisci il messaggio LaserScan con un singolo angolo e distanza
                                scan_msg = LaserScan()
                                scan_msg.header.stamp = self.get_clock().now().to_msg()
                                scan_msg.header.frame_id = "laser_frame"
                                scan_msg.angle_min = float(angle) * (3.14159 / 180.0)  # Converte l'angolo in radianti.
                                scan_msg.angle_max = float(angle) * (3.14159 / 180.0)
                                scan_msg.angle_increment = 0.0174533  # Incremento fisso per il singolo angolo
                                scan_msg.range_min = 0.1
                                scan_msg.range_max = 10.0
                                scan_msg.ranges = [1.0] * int((self.angle_max - self.angle_min) / self.angle_increment)
                                #scan_msg.ranges = [float(distance)]  # Lista con la singola distanza
                                scan_msg.intensities = []  # Intensity vuoto

                                # Pubblica il messaggio LaserScan
                                self.laserscan_publisher.publish(scan_msg)
                                self.get_logger().info(f"LaserScan pubblicato con angolo: {angle} e distanza: {distance}")
                            else:
                                self.get_logger().error("Chiavi 'angle' o 'distance' mancanti nel messaggio JSON.")
                        except json.JSONDecodeError as e:
                            self.get_logger().error(f"Errore durante il parsing del messaggio JSON: {e}")
                else:
                    self.get_logger().info("Connessione chiusa dal client.")
                    self.client_socket.close()  # Chiude la connessione con il client.
                    self.client_socket = None  # Resetta il socket del client.

            except Exception as e:
                self.get_logger().error(f"Errore durante la ricezione dei dati: {e}")
                # Gestisce eventuali errori durante la ricezione dei dati.

    def send_to_client_callback(self, msg):
        if self.client_socket:
            try:
                # Serializza le distanze come stringa separata da virgole
                serialized_data = ','.join(map(str, msg.ranges))  # Converte le distanze in stringa
                self.client_socket.sendall(serialized_data.encode('utf-8'))  # Invia i dati al client.
                self.get_logger().info("Messaggio LaserScan inviato al client.")
            except Exception as e:
                self.get_logger().error(f"Errore durante l'invio dei dati al client: {e}")
        else:
            self.get_logger().warn("Nessun client connesso. Impossibile inviare il messaggio.")
            # Logga un avviso se non c'è nessun client connesso.


    def destroy_node(self):
            # Chiudi il socket quando il nodo viene distrutto
            if self.client_socket:
                self.client_socket.close()  # Chiude il socket del client.
            self.server_socket.close()  # Chiude il socket del server.
            super().destroy_node()  # Distrugge il nodo ROS.

def main(args=None):
    rclpy.init(args=args)  # Inizializza ROS 2.
    node = TcpLaserScanNode()  # Crea un'istanza del nodo.
    try:
        rclpy.spin(node)  # Avvia il nodo ROS.
    except KeyboardInterrupt:
        pass  # Permette di interrompere l'esecuzione con Ctrl+C.
    finally:
        node.destroy_node()  # Distrugge il nodo quando il programma termina.
        rclpy.shutdown()  # Spegne ROS 2.

if __name__ == '__main__':
    main()  # Esegue la funzione main se il file è eseguito come script.
