#include <iostream>
#include <fstream>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include <cmath>
#include <chrono>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/gpio.h>
#include <linux/spi/spidev.h>
#include <sys/mman.h>

// Hardware-Konfiguration für RedPitaya
constexpr char SPI_DEVICE[] = "/dev/spidev0.0";
constexpr uint32_t GPIO_BASE = 0x40000000;
constexpr uint32_t GPIO_SIZE = 0x10000;
constexpr int BUTTON1_PIN = 16;  // E1
constexpr int BUTTON2_PIN = 17;  // E2
constexpr int ADC_CHANNEL_X = 0;
constexpr int ADC_CHANNEL_Y = 1;
constexpr int WEB_PORT = 8080;

// Globale Variablen für Datenmanagement
std::mutex data_mutex;
std::vector<float> x_positions;
std::vector<float> y_positions;
float current_x = 0.0f;
float current_y = 0.0f;
std::atomic<float> beam_stability{100.0f};
std::atomic<float> beam_intensity{100.0f};
std::atomic<bool> monitoring_active{true};

// GPIO-Mapping für direkten Hardware-Zugriff
volatile unsigned *gpio_map = nullptr;

// SPI-Konfiguration
struct spi_ioc_transfer spi_xfer;
uint8_t spi_tx_buf[3] = {0};
uint8_t spi_rx_buf[3] = {0};
int spi_fd = -1;

// Funktionen für SPI-Kommunikation
bool spi_init() {
    spi_fd = open(SPI_DEVICE, O_RDWR);
    if (spi_fd < 0) {
        perror("SPI device open failed");
        return false;
    }

    // SPI-Modus setzen
    uint8_t mode = SPI_MODE_0;
    if (ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) < 0) {
        perror("SPI mode set failed");
        close(spi_fd);
        return false;
    }

    // SPI-Geschwindigkeit setzen
    uint32_t speed = 1000000; // 1 MHz
    if (ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        perror("SPI speed set failed");
        close(spi_fd);
        return false;
    }

    return true;
}

float read_adc(int channel) {
    if (spi_fd < 0) return 0.0f;

    // ADC-Konfigurationsbyte
    spi_tx_buf[0] = 0x01; // Startbit
    spi_tx_buf[1] = (0x08 | channel) << 4; // Single-Ended, Channel
    spi_tx_buf[2] = 0x00;

    memset(&spi_xfer, 0, sizeof(spi_xfer));
    spi_xfer.tx_buf = (unsigned long)spi_tx_buf;
    spi_xfer.rx_buf = (unsigned long)spi_rx_buf;
    spi_xfer.len = 3;

    if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_xfer) < 0) {
        perror("SPI transfer failed");
        return 0.0f;
    }

    // ADC-Wert extrahieren
    int adc_value = ((spi_rx_buf[1] & 0x03) << 8) | spi_rx_buf[2];
    return (adc_value / 1023.0f) * 2.0f - 1.0f; // Normalisieren auf -1..1
}

// Funktionen für GPIO-Zugriff
bool gpio_init() {
    int mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd < 0) {
        perror("Memory device open failed");
        return false;
    }

    gpio_map = (volatile unsigned *)mmap(
        nullptr,
        GPIO_SIZE,
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        mem_fd,
        GPIO_BASE
    );

    close(mem_fd);

    if (gpio_map == MAP_FAILED) {
        perror("GPIO mmap failed");
        return false;
    }

    // GPIO-Pins als Eingang konfigurieren
    *(gpio_map + 1) &= ~(1 << BUTTON1_PIN); // Richtungsregister: Eingang
    *(gpio_map + 1) &= ~(1 << BUTTON2_PIN);

    return true;
}

bool read_gpio(int pin) {
    if (!gpio_map) return false;
    return (*(gpio_map + 0) & (1 << pin)) != 0; // Werteregister lesen
}

// WebSocket-Server Funktionen
void websocket_server() {
    int server_fd, new_socket;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);
    char buffer[1024] = {0};

    // Socket erstellen
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    // Socket-Optionen setzen
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(WEB_PORT);

    // Binden an Port
    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

    // Auf Verbindungen warten
    if (listen(server_fd, 3) < 0) {
        perror("listen");
        exit(EXIT_FAILURE);
    }

    std::cout << "WebSocket server listening on port " << WEB_PORT << std::endl;

    while (true) {
        // Neue Verbindung akzeptieren
        if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen)) < 0) {
            perror("accept");
            continue;
        }

        // HTTP-Header lesen
        read(new_socket, buffer, 1024);
        
        // Prüfen auf WebSocket-Upgrade-Anfrage
        if (strstr(buffer, "Upgrade: websocket")) {
            // WebSocket-Handshake durchführen (vereinfacht)
            const char *response = 
                "HTTP/1.1 101 Switching Protocols\r\n"
                "Upgrade: websocket\r\n"
                "Connection: Upgrade\r\n"
                "Sec-WebSocket-Accept: dGhlIHNhbXBsZSBub25jZQ==\r\n\r\n";
            
            send(new_socket, response, strlen(response), 0);
            
            // Hier könnte die eigentliche WebSocket-Kommunikation implementiert werden
            // Für dieses Beispiel senden wir kontinuierlich Daten
            while (true) {
                std::string data;
                {
                    std::lock_guard<std::mutex> lock(data_mutex);
                    data = "{\"x\":" + std::to_string(current_x) + 
                           ",\"y\":" + std::to_string(current_y) + 
                           ",\"stability\":" + std::to_string(beam_stability) + 
                           ",\"intensity\":" + std::to_string(beam_intensity) + "}";
                }
                
                // Vereinfachtes WebSocket-Frame senden
                uint8_t header[2] = {0x81, static_cast<uint8_t>(data.size())};
                send(new_socket, header, 2, 0);
                send(new_socket, data.c_str(), data.size(), 0);
                
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        }
        else {
            // HTTP-Anfrage - HTML-Seite ausliefern
            std::ifstream html_file("redpitaya_ui.html");
            if (html_file.is_open()) {
                std::string content((std::istreambuf_iterator<char>(html_file)), 
                    std::istreambuf_iterator<char>());
                
                std::string response = 
                    "HTTP/1.1 200 OK\r\n"
                    "Content-Type: text/html\r\n"
                    "Content-Length: " + std::to_string(content.size()) + "\r\n\r\n" + content;
                
                send(new_socket, response.c_str(), response.size(), 0);
            }
            else {
                const char *not_found = 
                    "HTTP/1.1 404 Not Found\r\n"
                    "Content-Type: text/plain\r\n"
                    "Content-Length: 18\r\n\r\n"
                    "File not found";
                send(new_socket, not_found, strlen(not_found), 0);
            }
        }
        
        close(new_socket);
    }
}

// Funktionen für Beam Monitoring
void beam_monitoring() {
    static int last_button1 = 0;
    static int last_button2 = 0;
    
    while (true) {
        // Buttons lesen
        int button1 = read_gpio(BUTTON1_PIN);
        int button2 = read_gpio(BUTTON2_PIN);
        
        // Button 1 gedrückt (Flankenerkennung)
        if (button1 && !last_button1 && monitoring_active) {
            // Messung durchführen
            float x = read_adc(ADC_CHANNEL_X);
            float y = read_adc(ADC_CHANNEL_Y);
            
            {
                std::lock_guard<std::mutex> lock(data_mutex);
                current_x = x;
                current_y = y;
                x_positions.push_back(x);
                y_positions.push_back(y);
                
                // Berechne Strahlstabilität und -intensität
                float distance = std::sqrt(x*x + y*y);
                beam_stability = 100.0f - (distance * 20.0f);
                beam_intensity = 100.0f - (distance * 15.0f);
                
                // Positionsdaten begrenzen
                if (x_positions.size() > 100) {
                    x_positions.erase(x_positions.begin());
                    y_positions.erase(y_positions.begin());
                }
            }
        }
        
        // Button 2 gedrückt (Flankenerkennung)
        if (button2 && !last_button2) {
            monitoring_active = !monitoring_active;
        }
        
        last_button1 = button1;
        last_button2 = button2;
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

int main() {
    // Hardware initialisieren
    if (!gpio_init()) {
        std::cerr << "GPIO initialization failed!" << std::endl;
        return 1;
    }
    
    if (!spi_init()) {
        std::cerr << "SPI initialization failed!" << std::endl;
        return 1;
    }
    
    std::cout << "RedPitaya Beam Monitoring System initialisiert" << std::endl;
    std::cout << "Verwende Tasten E1 (GPIO" << BUTTON1_PIN << ") und E2 (GPIO" << BUTTON2_PIN << ")" << std::endl;
    
    // Threads starten
    std::thread web_thread(websocket_server);
    std::thread monitor_thread(beam_monitoring);
    
    // Hauptsteuerung
    while (true) {
        // Systemstatus ausgeben
        std::cout << "Aktuelle Position: X=" << current_x << ", Y=" << current_y 
                  << " | Stabilität: " << beam_stability << "%, Intensität: " << beam_intensity << "%" << std::endl;
        
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    // Threads beenden (wird nie erreicht)
    web_thread.join();
    monitor_thread.join();
    
    return 0;
}
