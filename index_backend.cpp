#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <cmath>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <sys/mman.h>
#include <string.h>

// Hardware-Konfiguration
constexpr char SPI_DEVICE[] = "/dev/spidev0.0";
constexpr uint32_t GPIO_BASE = 0x40000000;
constexpr uint32_t GPIO_SIZE = 0x10000;
constexpr int BUTTON1_PIN = 16;  // E1
constexpr int BUTTON2_PIN = 17;  // E2
constexpr int ADC_CHANNEL_X = 0;
constexpr int ADC_CHANNEL_Y = 1;
constexpr int WEB_PORT = 8080;

// Globale Variablen
std::mutex data_mutex;
float current_x = 0.0f;
float current_y = 0.0f;
float beam_stability = 100.0f;
volatile unsigned *gpio_map = nullptr;
int spi_fd = -1;

// SPI Initialisierung
bool init_spi() {
    spi_fd = open(SPI_DEVICE, O_RDWR);
    if(spi_fd < 0) {
        std::cerr << "SPI Device konnte nicht geöffnet werden" << std::endl;
        return false;
    }

    uint8_t mode = SPI_MODE_0;
    uint32_t speed = 1000000;
    
    if(ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) < 0) {
        std::cerr << "SPI Mode konnte nicht gesetzt werden" << std::endl;
        close(spi_fd);
        return false;
    }
    
    if(ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        std::cerr << "SPI Geschwindigkeit konnte nicht gesetzt werden" << std::endl;
        close(spi_fd);
        return false;
    }

    return true;
}

// GPIO Initialisierung
bool init_gpio() {
    int mem_fd = open("/dev/mem", O_RDWR|O_SYNC);
    if(mem_fd < 0) {
        std::cerr << "/dev/mem konnte nicht geöffnet werden" << std::endl;
        return false;
    }

    gpio_map = (volatile unsigned*)mmap(NULL, GPIO_SIZE, PROT_READ|PROT_WRITE, 
                                      MAP_SHARED, mem_fd, GPIO_BASE);
    close(mem_fd);

    if(gpio_map == MAP_FAILED) {
        std::cerr << "GPIO Memory Mapping fehlgeschlagen" << std::endl;
        return false;
    }

    // GPIO-Pins als Eingang konfigurieren
    *(gpio_map + 1) &= ~(1 << BUTTON1_PIN);
    *(gpio_map + 1) &= ~(1 << BUTTON2_PIN);

    return true;
}

// ADC Lesen
float read_adc(int channel) {
    uint8_t tx[3] = {
        1, 
        static_cast<uint8_t>((0x08 | channel) << 4), // Explizite Konvertierung
        0
    };
    uint8_t rx[3] = {0};
    
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = 3,
    };

    if(ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
        std::cerr << "SPI Transfer fehlgeschlagen" << std::endl;
        return 0.0f;
    }

    return (((rx[1]&3) << 8) + rx[2]) / 1023.0f * 2.0f - 1.0f;
}

// Client-Handler
void handle_client(int client_socket) {
    char buffer[1024];
    int last_button1 = 0;
    int last_button2 = 0;

    while(true) {
        // Button Zustände lesen
        int button1 = (*(gpio_map + 0) & (1 << BUTTON1_PIN)) ? 1 : 0;
        int button2 = (*(gpio_map + 0) & (1 << BUTTON2_PIN)) ? 1 : 0;

        // Bei Button1 Druck messen
        if(button1 && !last_button1) {
            float x = read_adc(ADC_CHANNEL_X);
            float y = read_adc(ADC_CHANNEL_Y);
            
            std::lock_guard<std::mutex> lock(data_mutex);
            current_x = x;
            current_y = y;
            beam_stability = 100.0f - (std::sqrt(x*x + y*y) * 20.0f);
        }

        // Daten senden
        std::string data = 
            std::to_string(current_x) + "," +
            std::to_string(current_y) + "," +
            std::to_string(beam_stability) + "\n";
        
        if(send(client_socket, data.c_str(), data.size(), 0) <= 0) {
            break; // Verbindung geschlossen
        }

        // 50ms warten (20Hz Update)
        usleep(50000);
        
        last_button1 = button1;
        last_button2 = button2;
    }
    
    close(client_socket);
}

// Hauptfunktion
int main() {
    // Hardware initialisieren
    if(!init_gpio()) {
        std::cerr << "GPIO Initialisierung fehlgeschlagen!" << std::endl;
        return 1;
    }

    if(!init_spi()) {
        std::cerr << "SPI Initialisierung fehlgeschlagen!" << std::endl;
        return 1;
    }

    // Socket erstellen
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if(server_fd < 0) {
        std::cerr << "Socket konnte nicht erstellt werden" << std::endl;
        return 1;
    }

    // Socket Optionen setzen
    int opt = 1;
    if(setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        std::cerr << "Socket Optionen konnten nicht gesetzt werden" << std::endl;
        close(server_fd);
        return 1;
    }

    // Adresse binden
    sockaddr_in address = {
        .sin_family = AF_INET,
        .sin_port = htons(WEB_PORT),
        .sin_addr = {INADDR_ANY}
    };

    if(bind(server_fd, (sockaddr*)&address, sizeof(address)) < 0) {
        std::cerr << "Bind fehlgeschlagen" << std::endl;
        close(server_fd);
        return 1;
    }

    // Auf Verbindungen warten
    if(listen(server_fd, 5) < 0) {
        std::cerr << "Listen fehlgeschlagen" << std::endl;
        close(server_fd);
        return 1;
    }

    std::cout << "Server läuft auf Port " << WEB_PORT << std::endl;

    // Client-Verbindungen akzeptieren
    while(true) {
        int client_socket = accept(server_fd, nullptr, nullptr);
        if(client_socket < 0) {
            std::cerr << "Accept fehlgeschlagen" << std::endl;
            continue;
        }
        
        std::thread(handle_client, client_socket).detach();
    }

    close(server_fd);
    return 0;
}
