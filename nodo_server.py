#!/usr/bin/env python3           # interpreter line, che serve all'editor 
#                                 per capire che linguaggio viene utilizzato,
#                                 python3 in questo caso
import rclpy                     # importo la libreria di python per ros2
from rclpy.node import Node      # Per creare nodi servono linguaggi object-oriented, quindi qui creiamo il nodo fuori dal main. Importiamo dalla libreria rclpy la classe che serve per creare nodi
import socket
from std_msgs.msg import String

class TcpServerNode(Node):       #qui creo la classe e gli dico che tipo di classe è, in questo caso "Node"

    def __init__(self):          #creo il costruttore
        super().__init__('tcp_server_node')     #specifico il nome del nodo tra apici
        
        # Configura il server TCP
        self.HOST = '0.0.0.0'  
        self.PORT = 5169
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Publisher per i messaggi ricevuti
        self.publisher_ = self.create_publisher(String, 'tcp_messages', 10)

        # Sottoscrittore per inviare dati al client
        self.subscription = self.create_subscription(
            String,
            'tcp_send',
            self.send_to_client_callback,
            10
        )
        self.subscription  # Impedisce che venga eliminata dal garbage collector

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
                data = self.client_socket.recv(1024)
                if data:
                    message = data.decode('utf-8').strip()
                    self.get_logger().info(f"Messaggio ricevuto: {message}")

                    # Pubblica il messaggio su un topic ROS 2
                    msg = String()
                    msg.data = message
                    self.publisher_.publish(msg)
                else:
                    self.get_logger().info("Connessione chiusa dal client.")
                    self.client_socket.close()
                    self.client_socket = None

            except Exception as e:
                self.get_logger().error(f"Errore durante la ricezione dei dati: {e}")

    def send_to_client_callback(self, msg):
        if self.client_socket:
            try:
                # Invia il messaggio al client
                self.client_socket.sendall(msg.data.encode('utf-8'))
                self.get_logger().info(f"Messaggio inviato al client: {msg.data}")
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
    node = TcpServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
