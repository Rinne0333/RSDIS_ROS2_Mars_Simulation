#pragma once

#include <cerrno>
#include <poll.h>
#include <stdexcept>
#include <termios.h>
#include <unistd.h>

// Own the terminal configuration and restore it when this object is destroyed.
class Keyboard
{
public:
    Keyboard()
    {
        if (!isatty(STDIN_FILENO) || tcgetattr(STDIN_FILENO, &original_) == -1)
        {
            throw std::runtime_error("Run this node in an interactive terminal.");
        }

        termios settings = original_;
        settings.c_lflag &= ~(ICANON | ECHO);
        settings.c_iflag &= ~IXON;
        settings.c_cc[VMIN] = 0;
        settings.c_cc[VTIME] = 0;
        // Prevent Ctrl+Z from suspending the process with a nonzero command.
        settings.c_cc[VSUSP] = _POSIX_VDISABLE;

        if (tcsetattr(STDIN_FILENO, TCSANOW, &settings) == -1)
        {
            throw std::runtime_error("Could not configure the terminal.");
        }
    }

    ~Keyboard()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &original_);
    }

    Keyboard(const Keyboard &) = delete;
    Keyboard & operator=(const Keyboard &) = delete;

    // Return immediately: true means a byte was read, false means no input.
    bool read_key(char & key)
    {
        pollfd input{STDIN_FILENO, POLLIN, 0};
        const int ready = poll(&input, 1, 0);
        if (ready < 0)
        {
            if (errno == EINTR) {return false;}
            throw std::runtime_error("Keyboard poll failed.");
        }
        if (input.revents & (POLLHUP | POLLERR | POLLNVAL))
        {
            throw std::runtime_error("Terminal disconnected.");
        }
        if (ready == 0 || !(input.revents & POLLIN)) {return false;}

        const ssize_t count = read(STDIN_FILENO, &key, 1);
        if (count < 0 && errno != EINTR && errno != EAGAIN)
        {
            throw std::runtime_error("Keyboard read failed.");
        }
        return count == 1;
    }

private:
    termios original_{};
};
