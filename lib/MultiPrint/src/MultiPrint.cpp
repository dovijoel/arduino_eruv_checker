#ifndef MULTIPRINT_H
#define MULTIPRINT_H

#include <Print.h>
#include <vector>

class MultiPrint : public Print {
private:
    std::vector<Print *> printers;

public:
    // Add a Print object to the list
    void addPrinter(Print *printer) {
        printers.push_back(printer);
    }

    // Override the write method to send data to all printers
    virtual size_t write(uint8_t c) override {
        size_t totalBytesWritten = 0;
        for (Print *printer : printers) {
            if (printer) {
                totalBytesWritten += printer->write(c);
            }
        }
        return totalBytesWritten;
    }

    // Override the write method for buffers
    virtual size_t write(const uint8_t *buffer, size_t size) override {
        size_t totalBytesWritten = 0;
        for (Print *printer : printers) {
            if (printer) {
                totalBytesWritten += printer->write(buffer, size);
            }
        }
        return totalBytesWritten;
    }
};

#endif // MULTIPRINT_H