#include <Print.h>
#include <vector>
#include "MultiPrint.h"

void MultiPrint::addPrinter(Print *printer)
{
    printers.push_back(printer);
}

// Override the write method to send data to all printers
size_t MultiPrint::write(uint8_t c)
{
    size_t totalBytesWritten = 0;
    for (Print *printer : printers)
    {
        if (printer)
        {
            totalBytesWritten += printer->write(c);
        }
    }
    return totalBytesWritten;
}

// Override the write method for buffers
size_t MultiPrint::write(const uint8_t *buffer, size_t size)
{
    size_t totalBytesWritten = 0;
    for (Print *printer : printers)
    {
        if (printer)
        {
            totalBytesWritten += printer->write(buffer, size);
        }
    }
    return totalBytesWritten;
}