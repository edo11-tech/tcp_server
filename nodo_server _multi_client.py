#!/usr/bin/env python3       # interpreter line, che serve all'editor 
#                                 per capire che linguaggio viene utilizzato,
#                                 python3 in questo caso
import rclpy                      # importo la libreria di python per ros2
from rclpy.node import Node       # Per creare nodi servono linguaggi object-oriented, quindi qui creiamo il nodo fuori dal main. Importiamo dalla libreria rclpy la classe che serve per creare nodi. Importa la classe base Node per creare nodi ROS 2
import socket                     # Importa il modulo socket per la comunicazione TCP/IP
from std_msgs.msg import String   # Importa il tipo di messaggio String da ROS 2 per scambiare stringhe tra i nodi
from threading import Thread      # Importa la classe Thread per eseguire operazioni in parallelo

class TcpServerNode(Node):        #qui creo la classe e gli dico che tipo di classe è, in questo caso "Node"

    def __init__(self):           #creo il costruttore
        super().__init__('tcp_server_node')     #specifico il nome del nodo tra apici
        
        # Configura il server TCP usando parametri
        self.declare_parameter('host', '0.0.0.0')     # ip generico, valido per tutti gli ip
        self.declare_parameter('port', 5169)          # porta
        self.HOST = self.get_parameter('host').value  # Ottiene il valore del parametro 'host'
        self.PORT = self.get_parameter('port').value  # Ottiene il valore del parametro 'port'
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Publisher per i messaggi ricevuti
        self.publisher_ = self.create_publisher(String, 'tcp_messages', 10)    # Crea un publisher ROS 2 che pubblica messaggi di tipo String sul topic 'tcp_messages'  
                     #(type, nome, Qsize ovvero la quantità massima di messaggi)


        # Sottoscrittore per inviare dati al client
        self.subscription = self.create_subscription(
            String,       #type di messaggio del topic
            'tcp_send',   #nome del topic 
            self.send_to_client_callback,  #callback
            10            #Qsize ovvero la quantità massima di messaggi
        )        
        self.subscription

        self.clients = []      # Lista per memorizzare i socket dei client connessi
        self.running = True    # Flag per controllare se il server è attivo

        # Avvia il server TCP
        self.start_tcp_server()    

    def start_tcp_server(self):
        try:
            self.server_socket.bind((self.HOST, self.PORT))    
            self.server_socket.listen(5)  # Supporta fino a 5 connessioni in coda
            self.get_logger().info(f"Server TCP in ascolto su {self.HOST}:{self.PORT}...")

            # Accetta connessioni in un thread separato
            Thread(target=self.accept_clients, daemon=True).start()
        except Exception as e:
            self.get_logger().error(f"Errore durante l'avvio del server TCP: {e}")
            self.destroy_node()

    def accept_clients(self):    #abbiamo creato il server
         # Metodo per accettare connessioni dai client
        while self.running:
            try:
                client_socket, client_address = self.server_socket.accept()     # Blocca l'esecuzione fino a quando un client si connette, Restituisce il socket e l'indirizzo del client
                self.clients.append(client_socket)    # Aggiunge il socket del client alla lista dei client connessi
                self.get_logger().info(f"Connessione stabilita con {client_address}")  # Registra un messaggio informativo sulla connessione stabilita     #azione del nostro server

                # Gestisce la ricezione dati per ogni client in un thread separato
                Thread(target=self.receive_data, args=(client_socket,), daemon=True).start()   #Avvia un thread per ricevere dati dal client
            except Exception as e:
                self.get_logger().error(f"Errore durante l'accettazione di un client: {e}")

    def receive_data(self, client_socket):
         # Metodo per ricevere dati da un client specifico
        while self.running:
            try:
                data = client_socket.recv(1024)  #dimensione di byte che riceva dai client, 1024 in qst caso
                if data:    
                    message = data.decode('utf-8').strip()   # Decodifica il messaggio in una stringa e rimuove spazi inutili
                    self.get_logger().info(f"Messaggio ricevuto: {message}")  # Registra il messaggio ricevuto

                    # Pubblica il messaggio su un topic ROS 2
                    msg = String()
                    msg.data = message
                    self.publisher_.publish(msg)    # Pubblica il messaggio decodificato sul topic 'tcp_messages'
                else:
                    self.get_logger().info("Connessione chiusa dal client.")   # Registra un messaggio se il client chiude la connessione
                    self.clients.remove(client_socket)   # Rimuove il client dalla lista dei client attivi
                    client_socket.close()     # Chiude il socket del client
                    break
            except Exception as e:
                self.get_logger().error(f"Errore durante la ricezione dei dati: {e}")
                if client_socket in self.clients:
                    self.clients.remove(client_socket) # Rimuove il client dalla lista se c'è un errore
                client_socket.close()    # Chiude il socket in caso di errore
                break

    def send_to_client_callback(self, msg):    # Metodo per inviare un messaggio a tutti i client connessi
        for client_socket in self.clients:
            try:
                client_socket.sendall(msg.data.encode('utf-8'))  # Invia il messaggio al client codificandolo in UTF-8
                self.get_logger().info(f"Messaggio inviato al client: {msg.data}")     # Registra il messaggio inviato
            except Exception as e:
                self.get_logger().error(f"Errore durante l'invio dei dati al client: {e}")   # Registra un errore in caso di problemi nell'invio

    def destroy_node(self):
        # Ferma il server e chiudi tutte le connessioni
        self.running = False        # Ferma il ciclo principale del server
        for client_socket in self.clients:
            client_socket.close()    # Chiude ogni connessione attiva
        self.server_socket.close()   # Chiude il socket del server
        super().destroy_node()       # Esegue la distruzione della classe base Node


def main(args=None):         #metto l’argomento di default della funzione su None
    rclpy.init(args=args)    #necessario ad inizializzare le comunicazioni ROS
    node = TcpServerNode()   # Qui richiamo e creo effettivamente il nodo. Questo è il nodo che sta all’interno del programma, e tutti i nodi devono stare qui dentro
    try:
        rclpy.spin(node)     #spin serve per mantere il nodo attivo, finchè non viene killato il nodo.
    except KeyboardInterrupt:
        pass                 #gestisce l'interruzione manuale
    finally:
        node.destroy_node()  #Distrugge il nodo e rilascia le risorse
        rclpy.shutdown()     #necessario per chiudere le comunicazioni con ros2

if __name__ == '__main__':
    main()
