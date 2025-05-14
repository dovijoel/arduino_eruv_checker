#ifndef MULTIPRINT_H
#define MULTIPRINT_H

#include <Print.h>
#include <vector>

class MultiPrint : public Print {
private:
    std::vector<Print *> printers;

public:
    // Add a Print object to the list
    void addPrinter(Print *printer);

    // Override the write method to send data to all printers
    virtual size_t write(uint8_t c) override;

    // Override the write method for buffers
    virtual size_t write(const uint8_t *buffer, size_t size) override;
};

#endif // MULTIPRINT_H