/* fork-uart.c
 *
 * Test reception/transmission UART with forked parent/child processes.
 * Parent: lit le port série et affiche sur la console. Termine si '!' reçu.
 * Enfant: lit la console (stdin) et écrit sur le port série. Termine si 'q' entré.
 *
 * Usage: modifier la constante PORT_TTY ci-dessous si nécessaire (/dev/ttyS0, /dev/ttyUSB0, ...)
 *
 * Compile:
 *   gcc -Wall -O2 fork-uart.c -o fork-uart
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <string.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <signal.h>

#define PORT_TTY "/dev/ttyUSB2" /* <-- Remplacez par votre port série (ex: /dev/ttyUSB0) */
#define BAUDRATE B115200

static pid_t child_pid = -1;
static int serial_fd = -1;

/* Initialisation du port série dans une fonction séparée.
 * - Ouvre le device path (read/write, no controlling tty).
 * - Configure 115200 8N1, disable hardware flow control, no output processing.
 * - MODE: canonical vs non-canonical : ici on laisse les flags de ligne par défaut
 *   (mais on force VMIN/VTIME afin de lire 1 caractère et bloquer indéfiniment).
 * - VMIN = 1, VTIME = 0 -> read attend 1 octet sans timeout (attente infinie).
 */
int init_serial_port(const char *portPath) {
    int fd = open(portPath, O_RDWR | O_NOCTTY);
    if (fd == -1) {
        fprintf(stderr, "Erreur! ouverture de %s : %s\n", portPath, strerror(errno));
        return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        fprintf(stderr, "Erreur tcgetattr: %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    /* Baud rate */
    cfsetispeed(&tty, BAUDRATE);
    cfsetospeed(&tty, BAUDRATE);

    /* 8N1 */
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    /* No hardware flow control */
    tty.c_cflag &= ~CRTSCTS;

    /* Enable receiver and set local mode */
    tty.c_cflag |= (CLOCAL | CREAD);

    /* Disable software flow control */
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);

    /* Disable output processing */
    tty.c_oflag &= ~OPOST;

    /* Disable canonical echo/signals so that read() behavior is controlled by VMIN/VTIME.
       However the original snippet enabled ICANON; to satisfy the requirement "1 char with infinite delay"
       we must set VMIN=1 VTIME=0. Canonical mode buffers until newline: that would break single-char reads.
       So we disable ICANON to allow single-char raw reads. We also disable ECHO/ECHOE/ISIG. */
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    /* VMIN=1 and VTIME=0 => read() blocks until at least 1 byte is received (infinite timeout) */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 0;

    /* Apply attributes immediately */
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        fprintf(stderr, "Erreur tcsetattr: %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    /* Optional flush */
    tcflush(fd, TCIOFLUSH);

    return fd;
}

/* Handler so that parent can reap child if child exits early */
void sigchld_handler(int sig) {
    (void)sig;
    while (waitpid(-1, NULL, WNOHANG) > 0) { }
}

/* When parent receives '!' we will request child termination (if still alive) */
void terminate_child_if_alive(void) {
    if (child_pid > 0) {
        kill(child_pid, SIGTERM);
        /* wait for child to exit to avoid zombies */
        waitpid(child_pid, NULL, 0);
    }
}

int main(int argc, char *argv[]) {
    (void)argc;
    (void)argv;

    const char *portPath = PORT_TTY;

    printf("\nLecture Port Serie: %s\n", portPath);

    serial_fd = init_serial_port(portPath);
    if (serial_fd == -1) {
        fprintf(stderr, "Impossible d'initialiser le port série. Quitte.\n");
        return EXIT_FAILURE;
    }

    /* Install SIGCHLD handler to reap possible child exit */
    struct sigaction sa;
    sa.sa_handler = sigchld_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESTART | SA_NOCLDSTOP;
    sigaction(SIGCHLD, &sa, NULL);

    pid_t pid = fork();
    if (pid < 0) {
        fprintf(stderr, "Erreur fork: %s\n", strerror(errno));
        close(serial_fd);
        return EXIT_FAILURE;
    }

    if (pid == 0) {
        /* Enfant: lit la console (stdin) et écrit sur le port série.
         * Termine si l'utilisateur saisit 'q' (premier caractère de la ligne).
         */
        printf("Je suis le processus Fils, j'écrit sur le port série ce que j'entends sur la console (terminal)...\n");
        char inbuf[512];

        /* Make sure stdin is line-buffered so user can type */
        setvbuf(stdin, NULL, _IOLBF, 0);

        while (1) {
            if (fgets(inbuf, sizeof(inbuf), stdin) == NULL) {
                /* EOF or error on stdin -> exit */
                perror("fgets stdin");
                break;
            }

            /* If first non-newline char is 'q', exit child */
            if (inbuf[0] == 'q' && (inbuf[1] == '\n' || inbuf[1] == '\0')) {
                printf("Fin du Fils\n");
                /* Optionally notify parent by closing serial_fd (parent still has its own fd) */
                close(serial_fd);
                _exit(EXIT_SUCCESS);
            }

            /* Write the line to serial port */
            size_t to_write = strlen(inbuf);
            ssize_t w = write(serial_fd, inbuf, to_write);
            if (w < 0) {
                fprintf(stderr, "Erreur écriture port série: %s\n", strerror(errno));
                /* continue or break - we'll break to avoid busy failure loop */
                break;
            }
            /* flush to ensure bytes are sent immediately (optional) */
            tcdrain(serial_fd);
        }

        /* Child cleanup */
        close(serial_fd);
        _exit(EXIT_FAILURE);

    } else {
        /* Parent */
        child_pid = pid;
        printf("Je suis le processus Père, j'écrit sur la console (terminal) ce que j'entends sur le port série...\n");

        char rbuf[1024];
        ssize_t n;

        while (1) {
            /* read will block until at least 1 byte is received because VMIN=1 VTIME=0 */
            n = read(serial_fd, rbuf, sizeof(rbuf));
            if (n < 0) {
                if (errno == EINTR) continue; /* interrupted by signal, retry */
                fprintf(stderr, "Erreur lecture port série: %s\n", strerror(errno));
                break;
            } else if (n == 0) {
                /* Shouldn't happen with VMIN=1, but handle anyway */
                continue;
            }

            /* Print similar to required output:
             * processus Père: nombres d'octets recus : <n> --> <data>
             */
            /* Ensure we print printable characters safely */
            printf("processus Père: nombres d'octets recus : %zd --> ", n);
            /* Print exactly the received bytes (may include newline) */
            fwrite(rbuf, 1, n, stdout);
            printf("\n");

            /* Check if any received byte is '!' */
            int found_bang = 0;
            for (ssize_t i = 0; i < n; ++i) {
                if (rbuf[i] == '!') {
                    found_bang = 1;
                    break;
                }
            }

            if (found_bang) {
                /* Print the standalone '!' line like in sample */
                printf("!\n");
                /* request child termination if still alive and wait for it */
                terminate_child_if_alive();
                printf("Fin du Père\n");
                break;
            }

            /* If child exited earlier, parent may continue reading; if both should exit,
               you can modify behaviour. For now parent only exits on '!' per spec. */
        }

        /* Parent cleanup */
        close(serial_fd);
        return EXIT_SUCCESS;
    }
}