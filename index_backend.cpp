#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <sys/mman.h>

// Hardware Konfiguration
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
    if(spi_fd < 0) return false;

    uint8_t mode = SPI_MODE_0;
    uint32_t speed = 1000000;
    if(ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) < 0) return false;
    if(ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) return false;

    return true;
}

// GPIO Initialisierung
bool init_gpio() {
    int mem_fd = open("/dev/mem", O_RDWR|O_SYNC);
    if(mem_fd < 0) return false;

    gpio_map = (volatile unsigned*)mmap(NULL, GPIO_SIZE, PROT_READ|PROT_WRITE, 
                                      MAP_SHARED, mem_fd, GPIO_BASE);
    close(mem_fd);

    if(gpio_map == MAP_FAILED) return false;

    *(gpio_map + 1) &= ~(1 << BUTTON1_PIN);
    *(gpio_map + 1) &= ~(1 << BUTTON2_PIN);

    return true;
}

// ADC Lesen
float read_adc(int channel) {
    uint8_t tx[3] = {1, (0x08|channel)<<4, 0};
    uint8_t rx[3] = {0};
    
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = 3,
    };

    if(ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 0) return 0.0f;

    return (((rx[1]&3) << 8) + rx[2]) / 1023.0f * 2.0f - 1.0f;
}

// WebSocket Handler
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
            beam_stability = 100.0f - (sqrt(x*x + y*y) * 20.0f;
        }

        // Daten senden
        std::string data = 
            std::to_string(current_x) + "," +
            std::to_string(current_y) + "," +
            std::to_string(beam_stability) + "\n";
        
        send(client_socket, data.c_str(), data.size(), 0);

        // 50ms warten (20Hz Update)
        usleep(50000);
    }
}

// Hauptfunktion
int main() {
    if(!init_gpio() || !init_spi()) {
        std::cerr << "Hardware Initialisierung fehlgeschlagen!" << std::endl;
        return 1;
    }

    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in address = {
        .sin_family = AF_INET,
        .sin_port = htons(WEB_PORT),
        .sin_addr = {INADDR_ANY}
    };

    bind(server_fd, (sockaddr*)&address, sizeof(address));
    listen(server_fd, 5);

    std::cout << "Server läuft auf Port " << WEB_PORT << std::endl;

    while(true) {
        int client_socket = accept(server_fd, nullptr, nullptr);
        if(client_socket < 0) continue;
        
        std::thread(handle_client, client_socket).detach();
    }

    return 0;
}
