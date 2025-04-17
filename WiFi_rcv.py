
# import socket
# import hashlib
# from zeroconf import Zeroconf, ServiceBrowser, ServiceListener
# import time
# import sys

# class ESP32Listener(ServiceListener):
#     def __init__(self):
#         self.esp32_ip = None

#     def add_service(self, zc, type_, name):
#         if self.esp32_ip is None:
#             info = zc.get_service_info(type_, name)
#             self.esp32_ip = info.parsed_addresses()[0]

# def discover_esp32():
#     zc = Zeroconf()
#     listener = ESP32Listener()
#     browser = ServiceBrowser(zc, "_esp32video._tcp.local.", listener)
    
#     print("Searching for ESP32...")
#     for _ in range(30):  # 3-second timeout
#         if listener.esp32_ip:
#             zc.close()
#             print(f"Found ESP32 at {listener.esp32_ip}")
#             return listener.esp32_ip
#         time.sleep(0.1)
    
#     zc.close()
#     print("ESP32 not found")
#     return None

# def receive_video():
#     esp_ip = discover_esp32()
#     if not esp_ip:
#         return

#     with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
#         try:
#             # Connect to ESP32
#             s.settimeout(10)
#             print(f"Connecting to {esp_ip}:1234...")
#             s.connect((esp_ip, 1234))
#             print("Connected. Sending READY signal...")
#             s.sendall(b"READY")
            
#             # Prepare for file reception
#             file_hash = hashlib.md5()
#             total_bytes = 0
#             start_time = time.time()
#             file_size = None  # Will be updated if ESP32 sends it
            
#             with open("received_video.mp4", "wb") as f:
#                 while True:
#                     try:
#                         data = s.recv(1460)
#                         if not data:
#                             break  # Connection closed by ESP32
                        
#                         # Write data to file
#                         f.write(data)
#                         file_hash.update(data)
#                         total_bytes += len(data)
                        
#                         # Print progress
#                         if file_size:
#                             progress = (total_bytes / file_size) * 100
#                             sys.stdout.write(f"\rReceived: {total_bytes/(1024*1024):.2f} MB ({progress:.1f}%)")
#                         else:
#                             sys.stdout.write(f"\rReceived: {total_bytes/(1024*1024):.2f} MB")
#                         sys.stdout.flush()
                        
#                         # Reset timeout after first data
#                         s.settimeout(30)
#                     except socket.timeout:
#                         print("\nConnection timeout")
#                         break
            
#             # Verify MD5
#             server_md5 = None
#             s.settimeout(5)
#             try:
#                 while True:
#                     data = s.recv(64)
#                     if data.startswith(b"MD5:"):
#                         server_md5 = data[4:].decode().strip()
#                         break
#             except:
#                 pass
            
#             client_md5 = file_hash.hexdigest()
#             print(f"\nTransfer {'successful' if server_md5 == client_md5 else 'failed'}")
#             print(f"Server MD5: {server_md5}")
#             print(f"Client MD5: {client_md5}")
            
#         except Exception as e:
#             print(f"\nError: {str(e)}")
#         finally:
#             s.close()

# if __name__ == "__main__":
#     print("Starting video receiver...")
#     receive_video()



import socket
import hashlib
from zeroconf import Zeroconf, ServiceBrowser, ServiceListener
import time
import sys

class ESP32Listener(ServiceListener):
    def __init__(self):
        self.esp32_ip = None

    def add_service(self, zc, type_, name):
        if self.esp32_ip is None:
            info = zc.get_service_info(type_, name)
            self.esp32_ip = info.parsed_addresses()[0]

def discover_esp32():
    zc = Zeroconf()
    listener = ESP32Listener()
    browser = ServiceBrowser(zc, "_esp32video._tcp.local.", listener)
    
    print("Searching for ESP32...")
    for _ in range(30):  # 3-second timeout
        if listener.esp32_ip:
            zc.close()
            print(f"Found ESP32 at {listener.esp32_ip}")
            return listener.esp32_ip
        time.sleep(0.1)
    
    zc.close()
    print("ESP32 not found")
    return None

def receive_video():
    esp_ip = discover_esp32()
    if not esp_ip:
        return

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            # Connect to ESP32
            s.settimeout(10)
            print(f"Connecting to {esp_ip}:1234...")
            s.connect((esp_ip, 1234))
            print("Connected. Sending READY signal...")
            s.sendall(b"READY")
            
            # Prepare for file reception
            file_hash = hashlib.md5()
            total_bytes = 0
            start_time = None  # Timer will start with the first data packet
            file_size = None  # Will be updated if ESP32 sends it
            
            with open("received_video.mp4", "wb") as f:
                while True:
                    try:
                        data = s.recv(1460)
                        if not data:
                            break  # Connection closed by ESP32
                        
                        # Set start_time when the first packet is received
                        if start_time is None:
                            start_time = time.time()
                        
                        # Write data to file
                        f.write(data)
                        file_hash.update(data)
                        total_bytes += len(data)
                        
                        # Print progress
                        if file_size:
                            progress = (total_bytes / file_size) * 100
                            sys.stdout.write(f"\rReceived: {total_bytes/(1024*1024):.2f} MB ({progress:.1f}%)")
                        else:
                            sys.stdout.write(f"\rReceived: {total_bytes/(1024*1024):.2f} MB")
                        sys.stdout.flush()
                        
                        # Reset timeout after first data
                        s.settimeout(30)
                    except socket.timeout:
                        print("\nConnection timeout")
                        break
            
            # Calculate and print elapsed time if data was received
            if start_time is not None:
                elapsed_time = time.time() - start_time
                print(f"\nTime taken to receive all packets: {elapsed_time:.2f} seconds")
            else:
                print("\nNo data was received.")

            # Verify MD5
            server_md5 = None
            s.settimeout(5)
            try:
                while True:
                    data = s.recv(64)
                    if data.startswith(b"MD5:"):
                        server_md5 = data[4:].decode().strip()
                        break
            except:
                pass
            
            client_md5 = file_hash.hexdigest()
            print(f"Transfer {'successful' if server_md5 == client_md5 else 'failed'}")
            print(f"Server MD5: {server_md5}")
            print(f"Client MD5: {client_md5}")
            
        except Exception as e:
            print(f"\nError: {str(e)}")
        finally:
            s.close()

if __name__ == "__main__":
    print("Starting video receiver...")
    receive_video()
