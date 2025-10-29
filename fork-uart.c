/**
 * @file    fork-UART.c
 *
 * @brief Serial Port Programming in C (Fork practise)
 * Non Cannonical mode
 * @author  Samuel Crépeault
 * @date    2024-08-02
 */
#define _GNU_SOURCE

#include <stdio.h>
#include <fcntl.h>   // File Control Definitions
#include <termios.h> // POSIX Terminal Control Definitions
#include <unistd.h>  // UNIX Standard Definitions
#include <errno.h>   // ERROR Number Definitions
#include <sys/wait.h>
#include <stdlib.h>

const char *portTTY = "/dev/ttyUSB2"; // dynamique: revérifier le port ttyUSB au boot

// Prototypes
int vInitPortSerieEnfant(void);
int vInitPortSerieParent(void);
int fdEnfant; // File Descriptor
int fdParent; // File Descriptor

void main(void)
{
    pid_t pid;
    int i;
    int stopParent = 0;
    int stopEnfant = 0;
    fdEnfant = vInitPortSerieEnfant();
    fdParent = vInitPortSerieParent();
    pid = fork();

    if (pid < 0)
    {
        printf("Fork failed.\n");
    }
    else if (pid == 0)
    {
        ssize_t ucLongueur;
        char write_buffer[256];
        ssize_t bytes_written = 0;
        printf("Je suis le processus Enfant, j'écris sur le port série ce que j'entends sur le terminal.\n");
        while (!stopEnfant)
        {
            // Lire le terminal
            ucLongueur = read(STDIN_FILENO, write_buffer, sizeof(write_buffer));
            if (ucLongueur <= 0)
            {
                if (ucLongueur < 0)
                    perror("read");
            }
            // Arrête le processus enfant si 'q' est lu
            for (ssize_t i = 0; i < ucLongueur; i++)
            {
                if (write_buffer[i] == 'q')
                    stopEnfant = 1;
            }
            // Écrire sur le port série
            bytes_written = write(fdEnfant, write_buffer, ucLongueur);
            if (bytes_written < 0)
                perror("UART write error.\n");
            printf("\n");
        }
        printf("\nFin du processus enfant.\n");
        close(fdEnfant);
        exit(0);
    }
    else
    {
        printf("Je suis le processus Père, j'écris sur le terminal ce que j'entends sur le port série.\n");
        while (!stopParent)
        {
            // Lire sur port série
            //tcflush(fdParent, TCIFLUSH);
            char read_buffer[32];
            ssize_t bytes_read = 0;
            int i = 0;
            bytes_read = read(fdParent, &read_buffer, 32);
            if (bytes_read < 0)
            {
                perror("UART read error.\n");
            }

            // Arrête le parent si le caractère '!' est lu
            for (i = 0; i < bytes_read; i++)
            {
                if (read_buffer[i] == '!')
                    stopParent = 1;
            }

            // Écrire sur terminal
            printf("Processus Père : Nombre d'octets reçus: %d --> ", bytes_read);
            for (i = 0; i < bytes_read; i++)
            {
                printf("%c", read_buffer[i]);
            }
            printf("\n");
        }
        printf("\nFin du processus Père.\n");
        close(fdParent);
    }
}

int vInitPortSerieParent(void)
{
    // Opening the Serial Port
    fdParent = open(portTTY, O_RDWR | O_NOCTTY);
    // O_RDWR Read/Write access to serial port
    // O_NOCTTY - No terminal will control the process
    if (fdParent == -1) // Error Checking
    {
        perror("open");
    }
    else
        printf("\n Ouverture de %s reussie.\n ", portTTY);

    // Setting the Attributes of the serial port using termios structure
    struct termios SerialPortSettings;  // Create the structure
    tcgetattr(fdParent, &SerialPortSettings); // Get the current attributes of the Serial port
    // Setting the Baud rate
    cfsetispeed(&SerialPortSettings, B115200); // Set Read  Speed
    cfsetospeed(&SerialPortSettings, B115200); // Set Write Speed
    // 8N1 Mode
    SerialPortSettings.c_cflag &= ~PARENB;        // Disables the Parity Enable bit(PARENB),So No Parity
    SerialPortSettings.c_cflag &= ~CSTOPB;        // CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit
    SerialPortSettings.c_cflag &= ~CSIZE;         // Clears the mask for setting the data size
    SerialPortSettings.c_cflag |= CS8;            // Set the data bits = 8
    SerialPortSettings.c_cflag &= ~CRTSCTS;       // No Hardware flow Control
    SerialPortSettings.c_cflag |= CREAD | CLOCAL; // Enable receiver, Ignore Modem Control lines

    SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable XON/XOFF flow control both i/p and o/p                
    SerialPortSettings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // Non Canonical mode, Disable echo, Disable signal

    SerialPortSettings.c_oflag &= ~OPOST; // No Output Processing

    if ((tcsetattr(fdParent, TCSANOW, &SerialPortSettings)) != 0) // Set the attributes to the termios structure
        printf("\n  Erreur! configuration des attributs du port serie Parent\n");

    SerialPortSettings.c_cc[VMIN] = 1;   // Read at least X character(s)
    SerialPortSettings.c_cc[VTIME] = 50; // Wait 5sec (0 for indefinetly)

    return fdParent;
}

int vInitPortSerieEnfant(void)
{
    // Opening the Serial Port
    fdEnfant = open(portTTY, O_RDWR | O_NOCTTY);
    // O_RDWR Read/Write access to serial port
    // O_NOCTTY - No terminal will control the process
    if (fdEnfant == -1) // Error Checking
    {
        perror("open");
    }
    else
        printf("\n Ouverture de %s reussie.\n ", portTTY);

    // Setting the Attributes of the serial port using termios structure
    struct termios SerialPortSettings;  // Create the structure
    tcgetattr(fdEnfant, &SerialPortSettings); // Get the current attributes of the Serial port
    // Setting the Baud rate
    cfsetispeed(&SerialPortSettings, B115200); // Set Read  Speed
    cfsetospeed(&SerialPortSettings, B115200); // Set Write Speed
    // 8N1 Mode
    SerialPortSettings.c_cflag &= ~PARENB;        // Disables the Parity Enable bit(PARENB),So No Parity
    SerialPortSettings.c_cflag &= ~CSTOPB;        // CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit
    SerialPortSettings.c_cflag &= ~CSIZE;         // Clears the mask for setting the data size
    SerialPortSettings.c_cflag |= CS8;            // Set the data bits = 8
    SerialPortSettings.c_cflag &= ~CRTSCTS;       // No Hardware flow Control
    SerialPortSettings.c_cflag |= CREAD | CLOCAL; // Enable receiver, Ignore Modem Control lines

    SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable XON/XOFF flow control both i/p and o/p
    SerialPortSettings.c_lflag |= ICANON;                  // Canonical mode
    SerialPortSettings.c_lflag &= ~(ECHO | ECHOE | ISIG);  // Disable echo, Disable signal

    SerialPortSettings.c_oflag &= ~OPOST; // No Output Processing

    if ((tcsetattr(fdEnfant, TCSANOW, &SerialPortSettings)) != 0) // Set the attributes to the termios structure
        printf("\n  Erreur! configuration des attributs du port serie Enfant\n");

    SerialPortSettings.c_cc[VMIN] = 1;   // Read at least X character(s)
    SerialPortSettings.c_cc[VTIME] = 50; // Wait 5sec (0 for indefinetly)

    return fdEnfant;
}