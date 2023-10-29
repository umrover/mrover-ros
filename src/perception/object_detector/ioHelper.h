#ifndef __IO_HELPER_H__
#define __IO_HELPER_H__

#include <NvInfer.h>
#include <iostream>
#include <vector>

namespace nvinfer1 {

    std::ostream& operator<<(std::ostream& o, const ILogger::Severity severity);

    class Logger : public nvinfer1::ILogger {
    public:
        virtual void log(Severity severity, const char* msg) override {
            std::cerr << severity << ": " << msg << std::endl;
        }
    };

    template<typename T>
    struct Destroy {
        void operator()(T* t) const {
            t->destroy();
        }
    };

    std::string getBasename(std::string const& path);

    size_t readTensor(std::vector<std::string> const& tensorProtoPaths, std::vector<float>& buffer);

    void writeBuffer(void* buffer, size_t size, std::string const& path);

    std::string readBuffer(std::string const& path);

} // namespace nvinfer1

#endif /*__IO_HELPER_H__*/