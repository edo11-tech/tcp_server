import socket

# Configurazione del server TCP
HOST = '0.0.0.0'  # Ascolta su tutte le interfacce di rete
PORT = 5169       # Porta su cui il server è in ascolto

def start_server():
    # Creazione del socket TCP
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((HOST, PORT))
    server_socket.listen(1)  # Limita a una connessione simultanea
    print(f"Server avviato. In ascolto su {HOST} : {PORT}")

    try:
        while True:
            print("In attesa di una connessione...")
            # Accetta una connessione in entrata
            client_socket, client_address = server_socket.accept()
            print(f"Connessione accettata da {client_address}")

            try:
                while True:
                    # Ricevi i dati dal client
                    data = client_socket.recv(1024)  # Buffer di 1024 byte
                    if not data:
                        print("Connessione chiusa dal client.")
                        break

                    # Decodifica e stampa i dati ricevuti
                    lidar_data = data.decode('utf-8')
                    print("Dati ricevuti:")
                    print(lidar_data)

            except Exception as e:
                print(f"Errore nella gestione del client: {e}")
            finally:
                # Chiudi il socket del client
                client_socket.close()
                print("Connessione chiusa.")
    except KeyboardInterrupt:
        print("\nServer terminato.")
    finally:
        # Chiudi il socket del server
        server_socket.close()

if __name__ == "__main__":
    start_server()
